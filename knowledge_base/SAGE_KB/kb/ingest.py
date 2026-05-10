from __future__ import annotations
import os
import re
import requests
from bs4 import BeautifulSoup
from pypdf import PdfReader
import tiktoken
from django.conf import settings
from django.db import transaction
from django.contrib.postgres.search import SearchVector

from .models import Source, Chunk
from .embedders import get_embedder
embedder = get_embedder()

def _clean_text(s: str) -> str:
    s = re.sub(r"\s+", " ", s)
    return s.strip()

def _chunk_text(text: str, chunk_tokens: int = 700, overlap_tokens: int = 120) -> list[str]:
    enc = tiktoken.get_encoding("cl100k_base")
    toks = enc.encode(text)
    out = []
    start = 0
    while start < len(toks):
        end = min(start + chunk_tokens, len(toks))
        chunk = enc.decode(toks[start:end]).strip()
        if chunk:
            out.append(chunk)
        if end == len(toks):
            break
        start = max(0, end - overlap_tokens)
    return out

def _extract_url(url: str) -> str:
    r = requests.get(url, timeout=30, headers={"User-Agent": "SAGE-KB/1.0"})
    r.raise_for_status()
    soup = BeautifulSoup(r.text, "html.parser")
    for tag in soup(["script", "style", "nav", "footer", "header", "noscript"]):
        tag.decompose()
    return _clean_text(soup.get_text(separator=" "))

def _extract_pdf_pages(path: str) -> list[tuple[int, str]]:
    reader = PdfReader(path)
    pages = []
    for i, p in enumerate(reader.pages):
        t = _clean_text(p.extract_text() or "")
        if t:
            pages.append((i + 1, t))
    return pages

def ingest_source(source_id: int) -> None:
    src = Source.objects.get(id=source_id)

    if not src.enabled:
        with transaction.atomic():
            Chunk.objects.filter(source=src).delete()
            src.status = Source.STATUS_READY
            src.last_error = ""
            src.save(update_fields=["status", "last_error"])
        return

    # Mark processing
    src.status = Source.STATUS_PROCESSING
    src.last_error = ""
    src.save(update_fields=["status", "last_error"])

    try:
        chunks_payload: list[dict] = []

        if src.type == Source.TYPE_URL:
            raw = _extract_url(src.location)
            texts = _chunk_text(raw)
            chunks_payload = [{"page": None, "text": t} for t in texts]

        elif src.type == Source.TYPE_PDF:
            if not src.pdf_file:
                Chunk.objects.filter(source=src).delete()
                return
            pages = _extract_pdf_pages(src.pdf_file.path)

            for page_num, page_text in pages:
                page_chunks = _chunk_text(page_text)
                for t in page_chunks:
                    chunks_payload.append({"page": page_num, "text": t})

        else:
            raise ValueError("Unsupported source type")

        # If source extracts to nothing: clear old chunks
        if not chunks_payload:
            with transaction.atomic():
                Chunk.objects.filter(source=src).delete()
                src.status = Source.STATUS_READY
                src.save(update_fields=["status"])
            return

        # Embed in batches
        batch_size = 64
        embeddings: list[list[float]] = []
        texts_all = [c["text"] for c in chunks_payload]
        for i in range(0, len(texts_all), batch_size):
            embeddings.extend(embedder.embed_texts(texts_all[i:i+batch_size]))

        # Replace chunks atomically
        with transaction.atomic():
            Chunk.objects.filter(source=src).delete()

            bulk = []
            for idx, (c, emb) in enumerate(zip(chunks_payload, embeddings)):
                bulk.append(
                    Chunk(
                        source=src,
                        chunk_index=idx,
                        page=c["page"],
			section=c.get("section", ""),
                        text=c["text"],
			search_text=f"Title: {src.title or ''}\nSection: {c.get('section', '') or ''}\nPage: {c['page'] or ''}\nContent: {c['text']}",
                        embedding=emb,
                    )
                )
            Chunk.objects.bulk_create(bulk, batch_size=200)
            # bulk_create bypasses save(), so search_vector must be set manually
            Chunk.objects.filter(source=src).update(
                search_vector=SearchVector("search_text", config="english")
            )
            src.status = Source.STATUS_READY
            src.last_error = ""
            src.save(update_fields=["status", "last_error"])

    except Exception as e:
        # Don’t crash the admin save; mark failed
        src.status = Source.STATUS_FAILED
        src.last_error = str(e)
        src.save(update_fields=["status", "last_error"])
        raise