FROM golang:1.23-alpine AS builder

RUN apk add --no-cache git

WORKDIR /app

COPY go.mod go.sum ./
RUN go mod download

COPY . .
RUN go build -o rocketTelemetrySim ./cmd

FROM alpine:latest

WORKDIR /app
COPY --from=builder /app/rocketTelemetrySim /app/rocketTelemetrySim

EXPOSE 8086

ENTRYPOINT ["/app/rocketTelemetrySim"]
