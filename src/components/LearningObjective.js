import React from 'react';
import PropTypes from 'prop-types';

/**
 * Component for displaying learning objectives
 */
export default function LearningObjective({ children, id }) {
  return (
    <div className="learning-objective">
      <h4>🎯 Learning Objective {id ? `(${id})` : ''}</h4>
      <div>{children}</div>
    </div>
  );
}

LearningObjective.propTypes = {
  id: PropTypes.string,
  children: PropTypes.node.isRequired
};