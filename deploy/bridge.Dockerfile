FROM node:24-alpine

WORKDIR /app

COPY host/bridge/package.json host/bridge/package-lock.json ./
RUN npm ci

COPY host/bridge/tsconfig.json ./
COPY host/bridge/src ./src
COPY protocol/telemetry.yaml /app/protocol/telemetry.yaml

ENV NODE_ENV=production \
    PROTOCOL_SCHEMA=/app/protocol/telemetry.yaml \
    BRIDGE_HTTP_PORT=8080 \
    BRIDGE_HISTORY_PATH=/data/history.jsonl

RUN mkdir -p /data
VOLUME ["/data"]
EXPOSE 8080

HEALTHCHECK --interval=10s --timeout=3s --start-period=10s --retries=3 \
    CMD wget -qO- http://127.0.0.1:8080/health >/dev/null || exit 1

CMD ["npm", "start"]
