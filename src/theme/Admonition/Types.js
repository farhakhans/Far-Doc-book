import React from 'react';

// Custom admonition types for the Physical AI & Humanoid Robotics book
const customAdmonitions = {
  // Robotics-specific warning
  'robotics-warning': {
    infima: {
      type: 'danger',
      backgroundColor: 'rgba(255, 0, 0, 0.1)',
      borderColor: '#dc3545',
    },
    svg: (
      <svg
        fill="currentColor"
        height="24"
        viewBox="0 0 24 24"
        width="24"
        xmlns="http://www.w3.org/2000/svg">
        <path d="M0 0h24v24H0z" fill="none" />
        <path d="M12 2L1 21h22L12 2zm0 4.36l6.9 12.04H5.1L12 6.36zM11 10v4h2v-4h-2zm0 6v2h2v-2h-2z" />
      </svg>
    ),
    label: 'Robot Safety Warning',
  },
  // Physical AI concept highlight
  'physical-ai-tip': {
    infima: {
      type: 'success',
      backgroundColor: 'rgba(40, 167, 69, 0.1)',
      borderColor: '#28a745',
    },
    svg: (
      <svg
        fill="currentColor"
        height="24"
        viewBox="0 0 24 24"
        width="24"
        xmlns="http://www.w3.org/2000/svg">
        <path d="M0 0h24v24H0z" fill="none" />
        <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z" />
      </svg>
    ),
    label: 'Physical AI Tip',
  },
  // Hardware consideration
  'hardware-note': {
    infima: {
      type: 'info',
      backgroundColor: 'rgba(23, 162, 184, 0.1)',
      borderColor: '#17a2b8',
    },
    svg: (
      <svg
        fill="currentColor"
        height="24"
        viewBox="0 0 24 24"
        width="24"
        xmlns="http://www.w3.org/2000/svg">
        <path d="M0 0h24v24H0z" fill="none" />
        <path d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z" />
      </svg>
    ),
    label: 'Hardware Note',
  },
};

export default customAdmonitions;