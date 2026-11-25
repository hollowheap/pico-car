# Static web build of the dashboard, served by nginx. The bridge addresses are
# baked in at build time (Expo inlines EXPO_PUBLIC_* values), so compose.yaml
# passes the bridge URLs as build arguments.
FROM node:24-alpine AS build

WORKDIR /app/host/dashboard
COPY host/dashboard/package.json host/dashboard/package-lock.json ./
RUN npm ci

COPY protocol /app/protocol
COPY host/dashboard ./

ARG EXPO_PUBLIC_LINK_URL
ARG EXPO_PUBLIC_SERVICE_URL
ENV EXPO_PUBLIC_LINK_URL=$EXPO_PUBLIC_LINK_URL \
    EXPO_PUBLIC_SERVICE_URL=$EXPO_PUBLIC_SERVICE_URL \
    EXPO_PUBLIC_ALLOW_INSECURE=1 \
    EXPO_NO_TELEMETRY=1 \
    CI=1
RUN npx expo export --platform web --output-dir dist

FROM docker.io/library/nginx:1.29-alpine
COPY deploy/dashboard.nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/host/dashboard/dist /usr/share/nginx/html
EXPOSE 80
