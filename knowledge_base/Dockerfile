FROM python:3.13.5-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

# ---- system packages you need in the final image ----
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------
# rest of your file
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

COPY requirements.txt /app/
RUN uv pip install -r requirements.txt --system

# project source
COPY ./SAGE_KB/ /app/

# entrypoint script
RUN chmod +x /app/entrypoint.sh

EXPOSE 8004

# hand off to the shell script
CMD ["/app/entrypoint.sh"]
