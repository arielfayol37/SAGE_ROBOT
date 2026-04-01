from django.db import models
from pgvector.django import VectorField
from django.contrib.postgres.search import SearchVectorField
from django.db.models.signals import post_delete
from django.dispatch import receiver
import os

class Source(models.Model):
    TYPE_URL = "url"
    TYPE_PDF = "pdf"
    TYPE_CHOICES = [(TYPE_URL, "URL"), (TYPE_PDF, "PDF")]

    STATUS_READY = "ready"
    STATUS_PROCESSING = "processing"
    STATUS_FAILED = "failed"
    STATUS_CHOICES = [
        (STATUS_READY, "Ready"),
        (STATUS_PROCESSING, "Processing"),
        (STATUS_FAILED, "Failed"),
    ]

    type = models.CharField(max_length=8, choices=TYPE_CHOICES)
    title = models.CharField(max_length=255, blank=True)
    location = models.TextField(blank=True)  # URL for url sources
    pdf_file = models.FileField(upload_to="pdfs/", blank=True, null=True)

    enabled = models.BooleanField(default=True)

    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default=STATUS_READY)
    last_error = models.TextField(blank=True)

    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    def __str__(self):
        return self.title or f"{self.type}:{self.id}"
    
    def display_location(self) -> str:
        if self.type == self.TYPE_URL:
            return self.location
        if self.pdf_file:
            return self.pdf_file.path
        return ""

class Chunk(models.Model):
    source = models.ForeignKey(Source, on_delete=models.CASCADE, related_name="chunks")
    chunk_index = models.IntegerField()
    section = models.CharField(max_length=255, blank=True)
    page = models.IntegerField(blank=True, null=True)

    text = models.TextField()
    search_text = models.TextField(blank=True)
    embedding = VectorField(dimensions=1536)
    search_vector = SearchVectorField(null=True)

    class Meta:
        unique_together = [("source", "chunk_index")]
        indexes = [
            models.Index(fields=["source"]),
        ]
    def __str__(self):
        return f"Chunk {self.source_id}:{self.chunk_index}"

@receiver(post_delete, sender=Source)
def delete_pdf_file_on_source_delete(sender, instance, **kwargs):
    if instance.pdf_file:
        file_path = instance.pdf_file.path
        if os.path.isfile(file_path):
            os.remove(file_path)