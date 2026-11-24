FROM node:24-alpine AS dashboard
WORKDIR /app/host/dashboard
COPY host/dashboard/package.json host/dashboard/package-lock.json ./
RUN npm ci
COPY host/dashboard ./
COPY protocol /app/protocol
ARG EXPO_PUBLIC_ALLOW_INSECURE=0
RUN CI=1 EXPO_NO_TELEMETRY=1 npm run build:web

FROM node:24-alpine
WORKDIR /app/host/bridge

COPY host/bridge/package.json host/bridge/package-lock.json ./
RUN npm ci

COPY host/bridge/tsconfig.json ./
COPY host/bridge/src ./src
COPY protocol/telemetry.yaml /app/protocol/telemetry.yaml
COPY --from=dashboard /app/host/dashboard/dist /app/host/dashboard/dist

ENV NODE_ENV=production \
    BRIDGE_HTTP_HOST=0.0.0.0 \
    BRIDGE_HTTP_PORT=8080 \
    BRIDGE_HISTORY_PATH=/data/history.jsonl

RUN mkdir -p /data
VOLUME ["/data"]
EXPOSE 8080

CMD ["node", "--import", "tsx", "src/index.ts", "--station"]
