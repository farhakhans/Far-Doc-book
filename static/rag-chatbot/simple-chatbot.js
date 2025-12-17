// Simple Floating Chatbot Button
document.addEventListener('DOMContentLoaded', function() {
  console.log('Chatbot script loaded');

  // Create a simple button
  const button = document.createElement('button');
  button.id = 'simple-chatbot-btn';
  button.innerHTML = '💬';
  button.style.position = 'fixed';
  button.style.bottom = '20px';
  button.style.right = '20px';
  button.style.width = '60px';
  button.style.height = '60px';
  button.style.borderRadius = '50%';
  button.style.background = 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)';
  button.style.color = 'white';
  button.style.border = 'none';
  button.style.cursor = 'pointer';
  button.style.boxShadow = '0 4px 15px rgba(102, 126, 234, 0.4)';
  button.style.fontSize = '24px';
  button.style.zIndex = '10000';
  button.style.display = 'flex';
  button.style.alignItems = 'center';
  button.style.justifyContent = 'center';

  // Add click event
  button.addEventListener('click', function() {
    alert('Chatbot clicked!');
  });

  // Add to body
  document.body.appendChild(button);
  console.log('Chatbot button added to page');
});

// Check if script is being loaded
console.log('Floating chatbot script is running');