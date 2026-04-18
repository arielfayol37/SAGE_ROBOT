from django.urls import path
from .views import search, trigger_email_blast

urlpatterns = [
    path("search", search),
    path("send-emails", trigger_email_blast, name="kb-send-emails"),
]