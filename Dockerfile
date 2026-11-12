# Optional selected-language environment; native make remains the default.
FROM node:24.13.0-bookworm-slim AS node
FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates libstdc++6 make build-essential && rm -rf /var/lib/apt/lists/*
COPY --from=node /usr/local/bin/node /usr/local/bin/node
COPY --from=node /usr/local/lib/node_modules/npm /usr/local/lib/node_modules/npm
RUN ln -s ../lib/node_modules/npm/bin/npm-cli.js /usr/local/bin/npm && ln -s ../lib/node_modules/npm/bin/npx-cli.js /usr/local/bin/npx
WORKDIR /workspace
COPY . .
ENTRYPOINT ["npx", "--yes", "@clunkyard/cli"]
CMD ["match"]
