# Impulse2D documentation site

The documentation website for the **Impulse2D** 2D physics library, built with
[Docusaurus](https://docusaurus.io/).

## Prerequisites

- Node.js 18+ (developed with Node 22)
- npm 10+

## Install

```bash
npm install
```

## Local development

```bash
npm start
```

Starts a hot-reloading dev server at http://localhost:3000.

## Production build

```bash
npm run build
```

Outputs a static site to `build/`. Preview it locally with:

```bash
npm run serve
```

## Structure

- `docs/` — the documentation content (Introduction, Getting Started, Core Concepts,
  Collision, Forces, Tuning, The Demo, API Reference, Recipes).
- `sidebars.js` — the sidebar / information architecture.
- `docusaurus.config.js` — site config (title, navbar, footer). The GitHub URL is a
  placeholder — replace `GITHUB_URL` with the real repository URL before publishing.
- `src/` — homepage and theme customization.

The docs are derived from the engine source under `../src/Impulse2D`. When the public
API changes, update the relevant page (and the API Reference tables) to match.
