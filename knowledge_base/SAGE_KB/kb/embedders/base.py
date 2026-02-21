from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List

class BaseEmbedder(ABC):
    @property
    @abstractmethod
    def dim(self) -> int:
        ...

    @abstractmethod
    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        ...

    @abstractmethod
    def embed_query(self, query: str) -> List[float]:
        ...