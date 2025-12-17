import React from 'react';
import PropTypes from 'prop-types';

/**
 * Component for displaying practical exercises
 */
export default function PracticalExercise({ children, title="Exercise" }) {
  return (
    <div className="practical-exercise">
      <h4>🛠️ {title}</h4>
      <div>{children}</div>
    </div>
  );
}

PracticalExercise.propTypes = {
  title: PropTypes.string,
  children: PropTypes.node.isRequired
};