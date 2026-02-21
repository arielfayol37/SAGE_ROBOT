#!/usr/bin/env sh
set -e

# --- pre-start housekeeping ---
python manage.py makemigrations --noinput
python manage.py migrate --noinput
python manage.py collectstatic --noinput

# --- create superuser (if not exists) ---
python manage.py createsu

# --- launch your service ---
exec python manage.py runserver 0.0.0.0:8004
