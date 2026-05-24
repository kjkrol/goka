GO       = go
GOROOT  := $(shell go env GOROOT)

.PHONY: test

test:
	go test ./...
bench:
	go test -bench=. -benchmem ./...
