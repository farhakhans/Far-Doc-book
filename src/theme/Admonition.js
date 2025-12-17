import React from 'react';
import Admonition from '@theme-original/Admonition';
import customTypes from './Admonition/Types';

// Extend the default Admonition component with custom types
export default function AdmonitionWrapper(props) {
  const { type } = props;

  // If it's one of our custom types, use the original component with our custom types
  if (customTypes[type]) {
    return <Admonition {...props} />;
  }

  // Otherwise, use the original component as is
  return <Admonition {...props} />;
}