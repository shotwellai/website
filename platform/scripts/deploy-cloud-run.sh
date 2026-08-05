#!/usr/bin/env bash
set -euo pipefail

PROJECT="${GCP_PROJECT:-$(gcloud config get-value project 2>/dev/null)}"
REGION="${REGION:-us-central1}"
SERVICE="${SERVICE:-shotwell-platform}"

if [[ -z "${PROJECT}" ]]; then
  echo "Missing GCP project. Set GCP_PROJECT or run: gcloud config set project <project-id>" >&2
  exit 1
fi

# The bucket name stays out of this public repo. Pass it explicitly:
#   GCS_UPLOAD_BUCKET=<bucket> GCP_PROJECT=<project> bash scripts/deploy-cloud-run.sh
if [[ -z "${GCS_UPLOAD_BUCKET:-}" ]]; then
  echo "Missing GCS_UPLOAD_BUCKET. Set it in your environment before deploying." >&2
  exit 1
fi

gcloud run deploy "${SERVICE}" \
  --source . \
  --project "${PROJECT}" \
  --region "${REGION}" \
  --allow-unauthenticated \
  --update-env-vars "NODE_ENV=production,AUTH_STORE=postgres,PUBLIC_SITE_URL=https://shotwell.ai,AUTH_BASE_URL=https://auth.shotwell.ai,APP_BASE_URL=https://app.shotwell.ai,EMAIL_FROM=Shotwell <auth@shotwell.ai>,ADMIN_NOTIFICATION_EMAIL=founders@shotwell.ai,ALLOW_DEV_EMAIL_LOGIN=false,GCS_UPLOAD_BUCKET=${GCS_UPLOAD_BUCKET},GCS_UPLOAD_PREFIX=uploads,GCS_RESULTS_PREFIX=results" \
  --update-secrets "SESSION_SECRET=shotwell-platform-session-secret:latest,DATABASE_URL=shotwell-platform-database-url:latest,GOOGLE_CLIENT_ID=shotwell-platform-google-client-id:latest,GOOGLE_CLIENT_SECRET=shotwell-platform-google-client-secret:latest,RESEND_API_KEY=shotwell-platform-resend-api-key:latest"

cat <<EOF

Next:
1. Confirm these Secret Manager secrets exist and are readable by the Cloud Run service account:
   - shotwell-platform-session-secret
   - shotwell-platform-database-url
   - shotwell-platform-google-client-id
   - shotwell-platform-google-client-secret
   - shotwell-platform-resend-api-key, if enabling email login

2. Attach custom domains for:
   - auth.shotwell.ai
   - app.shotwell.ai

3. Add the resulting DNS records in Cloudflare.
EOF
