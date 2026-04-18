from django.http import JsonResponse
from django.conf import settings
from django.views.decorators.http import require_GET, require_POST
from django.contrib.postgres.search import SearchQuery, SearchRank
from pgvector.django import CosineDistance
from django.views.decorators.csrf import csrf_exempt
from django.core.mail import send_mass_mail
from .models import Chunk, EmailRecipient
from .embedders import get_embedder
import json


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

@csrf_exempt
@require_POST
def trigger_email_blast(request):
    try:
        data = json.loads(request.body.decode("utf-8"))
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON body."}, status=400)

    subject = (data.get("subject") or "").strip()
    body = (data.get("text") or "").strip()

    if not subject:
        return JsonResponse({"error": "Missing subject."}, status=400)

    if not body:
        return JsonResponse({"error": "Missing text."}, status=400)

    recipients = list(
        EmailRecipient.objects.filter(enabled=True).values_list("email", flat=True)
    )

    if not recipients:
        return JsonResponse({"error": "No enabled recipients found."}, status=400)

    from_email = getattr(settings, "DEFAULT_FROM_EMAIL", None)
    if not from_email:
        return JsonResponse({"error": "DEFAULT_FROM_EMAIL is not configured."}, status=500)

    messages = [(subject, body, from_email, [email]) for email in recipients]
    sent_count = send_mass_mail(messages, fail_silently=False)

    return JsonResponse({
        "success": True,
        "sent_count": sent_count,
        "recipients": recipients,
    })
