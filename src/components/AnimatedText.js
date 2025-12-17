import React, { useEffect } from 'react';

// Component to add animation effects to text elements
const AnimatedText = ({ children, type = 'heading', delay = 0, className = '' }) => {
  useEffect(() => {
    // Add animation classes when component mounts
    const elements = document.querySelectorAll(`.${className}`);
    elements.forEach((el, index) => {
      el.style.opacity = '0';
      el.style.transform = 'translateY(20px)';
      el.style.transition = 'opacity 0.6s ease-out, transform 0.6s ease-out';

      // Use a timeout to trigger the animation
      setTimeout(() => {
        el.style.opacity = '1';
        el.style.transform = 'translateY(0)';
      }, delay + (index * 100));
    });
  }, [className, delay]);

  return (
    <span className={className} style={{ transition: 'opacity 0.6s ease-out, transform 0.6s ease-out' }}>
      {children}
    </span>
  );
};

export default AnimatedText;