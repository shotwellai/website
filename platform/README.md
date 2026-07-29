# Shotwell Platform

This folder contains the authenticated app/backend service for:

- `auth.shotwell.ai`
- `app.shotwell.ai`

The existing marketing site at the repository root remains untouched and can continue to publish through GitHub Pages.

## Local Development

```sh
cd platform
npm install
cp .env.example .env
npm run dev
```

For local development, both `AUTH_BASE_URL` and `APP_BASE_URL` can point to `http://localhost:3000`. The same service is host-aware, so production can route `auth.shotwell.ai` and `app.shotwell.ai` to the same deployed backend.

## Environment

Required for production:

- `NODE_ENV=production`
- `PORT`
- `PUBLIC_SITE_URL`
- `AUTH_BASE_URL`
- `APP_BASE_URL`
- `SESSION_SECRET`
- `DATABASE_URL`
- `AUTH_STORE=postgres`

Required for Google SSO:

- `GOOGLE_CLIENT_ID`
- `GOOGLE_CLIENT_SECRET`

Optional:

- `GOOGLE_ALLOWED_DOMAIN`, for restricting Google SSO to a hosted domain.
- `ALLOW_DEV_EMAIL_LOGIN`, for local placeholder email login only.

## Current Auth Shape

The app already has the production route shape:

- `GET /login`
- `POST /login/email`
- `GET /login/google`
- `GET /auth/google/callback`
- `GET /auth/complete`
- `POST /logout`
- `GET /api/me`

The default local implementation uses an in-memory auth store so the app can run without external services. A Postgres-backed store is included for production.

```sh
cd platform
DATABASE_URL=... npm run db:migrate
AUTH_STORE=postgres npm run dev
```

Do not enable `AUTH_STORE=postgres` until `platform/migrations/001_auth.sql` has been applied to that database.

## Cloud Run

The included script deploys the service from this folder:

```sh
cd platform
./scripts/deploy-cloud-run.sh
```

The script assumes the active `gcloud` project unless `GCP_PROJECT` is set. It does not create DNS records. After Cloud Run has custom domains attached, add Cloudflare records for `auth` and `app` using the targets Cloud Run gives you.
