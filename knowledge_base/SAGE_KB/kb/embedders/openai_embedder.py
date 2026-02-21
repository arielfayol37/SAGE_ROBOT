from __future__ import annotations
import os
from typing import List
from openai import OpenAI
from .base import BaseEmbedder

class OpenAIEmbedder(BaseEmbedder):
    def __init__(self, model: str, dim: int):
        self._model = model
        self._dim = dim
        self._client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    @property
    def dim(self) -> int:
        return self._dim

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        resp = self._client.embeddings.create(model=self._model, input=texts)
        return [d.embedding for d in resp.data]

    def embed_query(self, query: str) -> List[float]:
        resp = self._client.embeddings.create(model=self._model, input=query)
        return resp.data[0].embedding