# Build stage
FROM golang:1.23-alpine AS builder

WORKDIR /app

# Copy go mod files
COPY go.mod go.sum ./
RUN go mod download

# Copy source code
COPY . .

# Build the application
RUN CGO_ENABLED=0 GOOS=linux go build -o /rocket-simulator ./cmd

# Final stage
FROM alpine:latest

RUN apk --no-cache add ca-certificates

WORKDIR /root/

# Copy binary from builder
COPY --from=builder /rocket-simulator .

# 8086 — метрики Prometheus, 8087 — REST API, WebSocket и веб-интерфейс.
# Интерфейс вшит в бинарник директивой go:embed, отдельный том не нужен.
EXPOSE 8086 8087

CMD ["./rocket-simulator"]

