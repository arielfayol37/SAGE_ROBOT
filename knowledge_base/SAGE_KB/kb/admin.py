from django.contrib import admin, messages
from .models import Source, Chunk, EmailRecipient
from .ingest import ingest_source
from django.db.models import Count

@admin.action(description="Ingest selected sources (replace chunks)")
def ingest_selected(modeladmin, request, queryset):
    ok, failed = 0, 0
    for src in queryset:
        try:
            ingest_source(src.id)
            ok += 1
        except Exception as e:
            failed += 1
            messages.error(request, f"Failed ingest for source {src.id}: {e}")

    if ok:
        messages.success(request, f"Ingested {ok} source(s).")
    if failed:
        messages.warning(request, f"{failed} source(s) failed.")

@admin.register(Source)
class SourceAdmin(admin.ModelAdmin):
    list_display = ("id", "type", "title", "enabled", "status", "updated_at")
    list_filter = ("type", "enabled", "status")
    search_fields = ("title", "location")
    actions = [ingest_selected]
    readonly_fields = ("status", "last_error", "created_at", "updated_at")

    def save_model(self, request, obj, form, change):
        # ingest only if new OR content changed OR enabled toggled
        if not change:
            should_ingest = True
        else:
            changed = set(form.changed_data)
            should_ingest = bool({"location", "pdf_file", "type", "enabled"} & changed)

        super().save_model(request, obj, form, change)

        if should_ingest:
            try:
                ingest_source(obj.id)
                messages.success(request, "Saved and ingested.")
            except Exception:
                # ingest_source already stored last_error + status=failed
                messages.error(request, f"Saved, but ingestion failed. See last_error.")
    
    def get_queryset(self, request):
        qs = super().get_queryset(request)
        return qs.annotate(_chunk_count=Count("chunks"))

    
    def chunk_count(self, obj):
        return obj._chunk_count
    chunk_count.admin_order_field = "_chunk_count"

@admin.register(Chunk)
class ChunkAdmin(admin.ModelAdmin):
    list_display = ("id", "source", "chunk_index", "page")
    list_filter = ("source__type",)
    search_fields = ("text",)

@admin.register(EmailRecipient)
class EmailRecipientAdmin(admin.ModelAdmin):
    list_display = ("email", "name", "enabled", "created_at")
    list_filter = ("enabled",)
    search_fields = ("email", "name")