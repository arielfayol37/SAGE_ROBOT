from django.http import JsonResponse
from django.views.decorators.http import require_GET
from pgvector.django import CosineDistance

from .models import Chunk
from .embedders import get_embedder

@require_GET
def search(request):
    q = (request.GET.get("q") or "").strip()
    k = int(request.GET.get("k") or 3)
    k = max(1, min(k, 20))

    if not q:
        return JsonResponse({"results": []})

    embedder = get_embedder()
    q_emb = embedder.embed_query(q)

    qs = (
        Chunk.objects
        .filter(source__enabled=True)
        .select_related("source")
        .annotate(distance=CosineDistance("embedding", q_emb))
        .order_by("distance")[:k]
    )

    results = []
    for ch in qs:
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