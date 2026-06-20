import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Zero Dependencies',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        A pure C# / .NET 9 library with no third-party packages. Drop it in as a
        project reference and start stepping a world.
      </>
    ),
  },
  {
    title: 'Rigid Bodies & Collision',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Circles and convex polygons, impulse-based response with restitution and
        Coulomb friction, and a pluggable broad phase.
      </>
    ),
  },
  {
    title: 'Forces & Tuning',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Gravity, drag, springs, attractors, buoyancy and wind &mdash; plus a
        simple <code>IForceGenerator</code> contract for your own.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
