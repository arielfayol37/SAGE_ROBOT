from __future__ import annotations
import os
from django.conf import settings
from .base import BaseEmbedder
from .openai_embedder import OpenAIEmbedder

def get_embedder() -> BaseEmbedder:
    backend = getattr(settings, "KB_EMBEDDER_BACKEND", "openai")

    if backend == "openai":
        model = getattr(settings, "KB_OPENAI_EMBED_MODEL", "text-embedding-3-small")
        dim = int(getattr(settings, "KB_EMBED_DIM", 1536))
        return OpenAIEmbedder(model=model, dim=dim)

    if backend == "local":
        # later: import and return LocalEmbedder()
        from .local_stub import LocalEmbedder
        return LocalEmbedder()

    raise ValueError(f"Unknown KB_EMBEDDER_BACKEND={backend}")