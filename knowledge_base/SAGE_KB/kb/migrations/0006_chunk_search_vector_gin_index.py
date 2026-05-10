from django.contrib.postgres.indexes import GinIndex
from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ("kb", "0005_emailrecipient"),
    ]

    operations = [
        migrations.AddIndex(
            model_name="chunk",
            index=GinIndex(fields=["search_vector"], name="chunk_search_vector_gin"),
        ),
    ]
