from django.http import JsonResponse
from django.views.decorators.http import require_GET
from django.contrib.postgres.search import SearchQuery, SearchRank
from pgvector.django import CosineDistance

from .models import Chunk
from .embedders import get_embedder


@require_GET
def search(request):
    q = (request.GET.get("q") or "").strip()
    k = int(request.GET.get("k") or 5)
    k = max(1, min(k, 20))

    if not q:
        return JsonResponse({"results": []})

    embedder = get_embedder()
    q_emb = embedder.embed_query(q)

    # Retrieve more candidates than final k
    candidate_k = max(10, k * 3)

    # Vector search
    vector_qs = (
        Chunk.objects
        .filter(source__enabled=True)
        .select_related("source")
        .annotate(distance=CosineDistance("embedding", q_emb))
        .order_by("distance")[:candidate_k]
    )

    vector_results = []
    for ch in vector_qs:
        vector_results.append({
            "id": ch.id,
            "chunk": ch,
            "vector_score": 1.0 - float(ch.distance),  # rough similarity
            "keyword_score": 0.0,
        })

    # Keyword search
    search_query = SearchQuery(q)
    keyword_qs = (
        Chunk.objects
        .filter(source__enabled=True)
        .select_related("source")
        .annotate(rank=SearchRank("search_vector", search_query))
        .filter(rank__gt=0.0)
        .order_by("-rank")[:candidate_k]
    )

    keyword_results = []
    for ch in keyword_qs:
        keyword_results.append({
            "id": ch.id,
            "chunk": ch,
            "vector_score": 0.0,
            "keyword_score": float(ch.rank),
        })

    # Merge by chunk id
    merged = {}

    for r in vector_results:
        merged[r["id"]] = r

    for r in keyword_results:
        if r["id"] in merged:
            merged[r["id"]]["keyword_score"] = r["keyword_score"]
        else:
            merged[r["id"]] = r

    # Simple combined score
    # You can tune these weights later
    combined = []
    for r in merged.values():
        score = 0.7 * r["vector_score"] + 0.3 * r["keyword_score"]
        combined.append((score, r["chunk"]))

    combined.sort(key=lambda x: x[0], reverse=True)
    top_chunks = [ch for _, ch in combined[:k]]

    results = []
    for ch in top_chunks:
        src = ch.source
        location = src.location if src.type == "url" else (src.pdf_file.url if src.pdf_file else "")
        results.append({
            "chunk_id": ch.id,
            "text": ch.text,
            "citation": {
                "source_id": src.id,
                "title": src.title,
                "type": src.type,
                "location": location,
                "page": ch.page,
                "section": ch.section,
            }
        })

    return JsonResponse({"results": results})