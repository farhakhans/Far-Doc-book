import React, { useEffect } from 'react';
import { initAnimations } from '../utils/animateHeadings';

// Component to initialize animations when the page loads
const AnimationInitializer = () => {
  useEffect(() => {
    // Initialize animations when component mounts
    initAnimations();

    // Set up a MutationObserver to handle dynamically added content
    const observer = new MutationObserver(() => {
      initAnimations();
    });

    // Watch for changes to the body and main content areas
    observer.observe(document.body, {
      childList: true,
      subtree: true
    });

    // Clean up observer on unmount
    return () => {
      observer.disconnect();
    };
  }, []);

  return null; // This component doesn't render anything
};

export default AnimationInitializer;