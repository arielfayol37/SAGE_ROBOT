# kb/management/commands/createsu.py
import os
from django.contrib.auth import get_user_model
from django.core.management.base import BaseCommand


class Command(BaseCommand):
    help = "Create a superuser using environment variables"

    def handle(self, *args, **kwargs):
        User = get_user_model()

        username = os.getenv("DJANGO_SU_USERNAME", "admin")
        email = os.getenv("DJANGO_SU_EMAIL", "admin@example.com")
        password = os.getenv("DJANGO_SU_PASSWORD", "adminpass")

        if not User.objects.filter(username=username).exists():
            User.objects.create_superuser(username=username, email=email, password=password)
            self.stdout.write(self.style.SUCCESS(f"Superuser '{username}' created"))
        else:
            self.stdout.write(self.style.WARNING(f"Superuser '{username}' already exists"))
