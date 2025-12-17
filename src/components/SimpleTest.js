import React from 'react';

const SimpleTest = () => {
  return (
    <div
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        backgroundColor: 'red',
        color: 'white',
        width: '60px',
        height: '60px',
        borderRadius: '50%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 10000,
        fontSize: '24px',
        fontWeight: 'bold',
        border: '2px solid yellow',
        boxShadow: '0 4px 8px rgba(0,0,0,0.3)'
      }}
      onClick={() => alert('Test button clicked!')}
    >
      T
    </div>
  );
};

export default SimpleTest;