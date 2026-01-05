import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI',
    description: (
      <>
        Connecting AI cognition with real-world robotic action through hands-on simulation and implementation.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Building autonomous humanoid robots capable of understanding and executing complex voice commands.
      </>
    ),
  },
  {
    title: 'Complete Learning Path',
    description: (
      <>
        From ROS 2 nervous system to AI-brain integration, comprehensive modules for mastering robotics.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
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