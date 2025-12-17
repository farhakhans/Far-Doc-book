import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI Focus',
    Svg: require('../../static/img/robot-arm.svg').default,
    description: (
      <>
        Learn to bridge the gap between digital AI and physical robot control,
        focusing on embodied intelligence and real-world applications.
      </>
    ),
  },
  {
    title: 'Hands-on Learning',
    Svg: require('../../static/img/code.svg').default,
    description: (
      <>
        Practical exercises and projects that let you implement ROS 2 systems,
        simulation environments, and Vision-Language-Action systems.
      </>
    ),
  },
  {
    title: 'Industry Standard Tools',
    Svg: require('../../static/img/gear.svg').default,
    description: (
      <>
        Master industry-standard tools like ROS 2, Gazebo, NVIDIA Isaac,
        and modern AI frameworks used in robotics today.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {Svg ? <Svg className={styles.featureSvg} role="img" /> : <div className={styles.featureSvgPlaceholder}></div>}
      </div>
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