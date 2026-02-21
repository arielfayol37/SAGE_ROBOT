from __future__ import annotations
from typing import List
from .base import BaseEmbedder

class LocalEmbedder(BaseEmbedder):
    """
    Placeholder. Later you can implement with sentence-transformers on Jetson:
      - load model once at startup
      - encode() texts to vectors
    """
    def __init__(self):
        raise NotImplementedError("Local embedder not wired yet")

    @property
    def dim(self) -> int:
        raise NotImplementedError

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        raise NotImplementedError

    def embed_query(self, query: str) -> List[float]:
        raise NotImplementedError