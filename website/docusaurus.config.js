// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Placeholder repository URL. Replace with the real GitHub URL when published.
const GITHUB_URL = 'https://github.com/your-org/physics-engine';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Impulse2D',
  tagline: 'A small, dependency-free 2D rigid-body physics engine for C# / .NET 9',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'physics-engine',

  // Be lenient about cross-links while the docs evolve.
  onBrokenLinks: 'warn',

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: 'docs',
          // No edit URL — point this at your repo to enable "edit this page" links.
        },
        // Blog is unused for an API documentation site.
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Impulse2D',
        logo: {
          alt: 'Impulse2D Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Docs',
          },
          {
            to: '/docs/api-reference/overview',
            label: 'API',
            position: 'left',
          },
          {
            href: GITHUB_URL,
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {label: 'Introduction', to: '/docs/introduction/overview'},
              {label: 'Getting Started', to: '/docs/getting-started/installation'},
              {label: 'API Reference', to: '/docs/api-reference/overview'},
            ],
          },
          {
            title: 'Guides',
            items: [
              {label: 'Core Concepts', to: '/docs/core-concepts/world'},
              {label: 'Collision', to: '/docs/collision/detection'},
              {label: 'Forces', to: '/docs/forces/overview'},
            ],
          },
          {
            title: 'More',
            items: [
              {label: 'Recipes', to: '/docs/recipes/bouncing-balls'},
              {label: 'GitHub', href: GITHUB_URL},
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Impulse2D. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['csharp', 'bash'],
      },
    }),
};

export default config;
