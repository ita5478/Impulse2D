// @ts-check

/**
 * Explicit sidebar for the Impulse2D documentation.
 * The information architecture mirrors the engine's modules.
 *
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  docsSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'introduction/overview',
        'introduction/architecture',
        'introduction/conventions',
        'introduction/limitations',
      ],
    },
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'getting-started/installation',
        'getting-started/first-simulation',
        'getting-started/step-loop',
      ],
    },
    {
      type: 'category',
      label: 'Core Concepts',
      items: [
        'core-concepts/world',
        'core-concepts/rigidbody',
        'core-concepts/materials',
        'core-concepts/shapes',
        'core-concepts/coordinates',
      ],
    },
    {
      type: 'category',
      label: 'Collision',
      items: [
        'collision/detection',
        'collision/broad-phase',
      ],
    },
    {
      type: 'category',
      label: 'Forces',
      items: [
        'forces/overview',
        'forces/generators',
        'forces/custom',
      ],
    },
    {
      type: 'category',
      label: 'Tuning',
      items: [
        'tuning/world-settings',
      ],
    },
    {
      type: 'category',
      label: 'The Demo',
      items: [
        'demo/running',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api-reference/overview',
        'api-reference/world',
        'api-reference/rigidbody',
        'api-reference/shapes',
        'api-reference/collision',
        'api-reference/forces',
        'api-reference/math',
      ],
    },
    {
      type: 'category',
      label: 'Recipes',
      items: [
        'recipes/bouncing-balls',
        'recipes/box-stack',
        'recipes/spring-chain',
        'recipes/attractor',
      ],
    },
  ],
};

export default sidebars;
