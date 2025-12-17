// Sidebar RAG Chatbot Widget
class SidebarRAGChatbot {
  constructor(options = {}) {
    this.mcpApiUrl = options.mcpApiUrl || 'http://localhost:5010'; // MCP adapter URL
    this.isOpen = true; // Always open by default
    this.initializeWidget();
    this.initializeEventListeners();
    this.addStyles();
  }

  addStyles() {
    // Add the typing animation CSS if not already present
    if (!document.querySelector('#typing-animation')) {
      const style = document.createElement('style');
      style.id = 'typing-animation';
      style.textContent = `
        @keyframes typing {
          0%, 60%, 100% { transform: translateY(0); }
          30% { transform: translateY(-5px); }
        }
      `;
      document.head.appendChild(style);
    }

    // Add body class to adjust content padding
    document.body.classList.add('sidebar-chatbot-enabled');
  }

  initializeWidget() {
    // Create the widget HTML
    const widgetHTML = `
      <div id="sidebar-chatbot" class="sidebar-chatbot">
        <!-- Chatbot Header -->
        <div class="sidebar-chatbot-header">
          <div class="sidebar-chatbot-title">
            <div class="sidebar-chatbot-icon">
              <i class="fas fa-robot"></i>
            </div>
            <div class="sidebar-chatbot-name">
              <h3>AI Assistant</h3>
              <p>Documentation Helper</p>
            </div>
          </div>
          <button id="sidebar-chatbot-close" class="sidebar-chatbot-close">
            <i class="fas fa-times"></i>
          </button>
        </div>

        <!-- Chat Messages Area -->
        <div id="sidebar-chat-messages" class="sidebar-chatbot-messages">
          <div class="sidebar-welcome-message">
            <h4><i class="fas fa-robot"></i> Welcome to AI Assistant</h4>
            <p>Ask me anything about the documentation!</p>
          </div>
        </div>

        <!-- Input Area -->
        <div class="sidebar-input-area">
          <div class="sidebar-input-container">
            <textarea
              id="sidebar-user-input"
              class="sidebar-user-input"
              placeholder="Ask about documentation..."
            ></textarea>
            <button id="sidebar-send-button" class="sidebar-send-button">
              <i class="fas fa-paper-plane"></i>
            </button>
          </div>
        </div>
      </div>
    `;

    // Add the widget to the page
    document.body.insertAdjacentHTML('beforeend', widgetHTML);

    // Store references to elements
    this.container = document.getElementById('sidebar-chatbot');
    this.closeButton = document.getElementById('sidebar-chatbot-close');
    this.chatMessages = document.getElementById('sidebar-chat-messages');
    this.userInput = document.getElementById('sidebar-user-input');
    this.sendButton = document.getElementById('sidebar-send-button');

    // Force the element to be positioned correctly
    if (this.container) {
      this.container.style.position = 'fixed';
      this.container.style.top = '0';
      this.container.style.right = '0';
      this.container.style.width = '400px';
      this.container.style.height = '100vh';
      this.container.style.zIndex = '9998';
      this.container.style.display = 'flex';
      this.container.style.flexDirection = 'column';
    }
  }

  initializeEventListeners() {
    // Close button functionality
    this.closeButton.addEventListener('click', () => this.toggleChatbot());

    // Send message
    this.sendButton.addEventListener('click', () => this.sendMessage());
    this.userInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        this.sendMessage();
      }
    });

    // Auto-resize textarea
    this.userInput.addEventListener('input', () => {
      this.autoResizeTextarea();
    });
  }

  autoResizeTextarea() {
    this.userInput.style.height = 'auto';
    this.userInput.style.height = Math.min(this.userInput.scrollHeight, 100) + 'px';
  }

  toggleChatbot() {
    this.isOpen = !this.isOpen;
    if (this.isOpen) {
      this.container.classList.remove('hidden');
      document.body.classList.add('sidebar-chatbot-enabled');
    } else {
      this.container.classList.add('hidden');
      document.body.classList.remove('sidebar-chatbot-enabled');
    }
  }

  async sendMessage() {
    const message = this.userInput.value.trim();
    if (!message) return;

    // Remove welcome message if it's the first interaction
    this.removeWelcomeMessage();

    // Add user message to chat
    this.addMessage('user', message);
    this.userInput.value = '';
    this.userInput.style.height = '45px';

    try {
      // Show typing indicator
      const typingIndicator = this.showTypingIndicator();

      // Use MCP adapter to execute the ask_documentation tool
      const response = await fetch(`${this.mcpApiUrl}/execute`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          tool_name: 'ask_documentation',
          arguments: {
            query: message,
            context_size: 3
          }
        })
      });

      // Remove typing indicator
      this.removeTypingIndicator(typingIndicator);

      const data = await response.json();

      if (response.ok && data.success) {
        // Add AI response to chat with context sources
        this.addMessage('ai', data.response, data.context_sources);
      } else {
        this.addMessage('ai', `Error: ${data.error || data.message || 'Unknown error occurred'}`);
      }
    } catch (error) {
      // Remove typing indicator
      this.removeTypingIndicator(typingIndicator);
      this.addMessage('ai', `Connection error: ${error.message}`);
    }
  }

  removeWelcomeMessage() {
    const welcomeMessage = this.chatMessages.querySelector('.sidebar-welcome-message');
    if (welcomeMessage) {
      welcomeMessage.remove();
    }
  }

  showTypingIndicator() {
    const typingDiv = document.createElement('div');
    typingDiv.className = 'sidebar-message ai';
    typingDiv.innerHTML = `
      <div class="sidebar-typing-indicator">
        <div class="sidebar-typing-dots">
          <div class="sidebar-typing-dot"></div>
          <div class="sidebar-typing-dot"></div>
          <div class="sidebar-typing-dot"></div>
        </div>
      </div>
    `;

    this.chatMessages.appendChild(typingDiv);
    this.scrollToBottom();
    return typingDiv;
  }

  removeTypingIndicator(typingIndicator) {
    if (typingIndicator && typingIndicator.parentNode) {
      typingIndicator.remove();
    }
  }

  addMessage(sender, text, contextSources = null) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `sidebar-message ${sender}`;

    // Create message header with icon
    const headerDiv = document.createElement('div');
    headerDiv.className = 'sidebar-message-header';

    const iconDiv = document.createElement('div');
    iconDiv.className = `sidebar-message-icon sidebar-${sender}-icon`;
    iconDiv.innerHTML = sender === 'user' ? '<i class="fas fa-user"></i>' : '<i class="fas fa-robot"></i>';

    const nameSpan = document.createElement('span');
    nameSpan.textContent = sender === 'user' ? 'You' : 'AI Assistant';

    headerDiv.appendChild(iconDiv);
    headerDiv.appendChild(nameSpan);

    // Create message content
    const contentDiv = document.createElement('div');
    contentDiv.className = 'sidebar-message-content';
    contentDiv.innerHTML = this.formatMessage(text);

    messageDiv.appendChild(headerDiv);
    messageDiv.appendChild(contentDiv);

    // Add context sources if provided
    if (contextSources && contextSources.length > 0) {
      const contextDiv = document.createElement('div');
      contextDiv.className = 'sidebar-message-context';
      contextDiv.innerHTML = '<strong>Referenced documents:</strong><br>' +
        contextSources.map(ctx =>
          `<div class="sidebar-context-source"><em>${ctx.title}</em> (${ctx.filepath}) - Relevance: ${(ctx.similarity * 100).toFixed(1)}%</div>`
        ).join('');

      messageDiv.appendChild(contextDiv);
    }

    this.chatMessages.appendChild(messageDiv);
    this.scrollToBottom();
  }

  formatMessage(text) {
    // Simple formatting - convert newlines to <br>
    return text.replace(/\n/g, '<br>');
  }

  scrollToBottom() {
    this.chatMessages.scrollTop = this.chatMessages.scrollHeight;
  }
}

// Auto-initialize the widget when the page loads
document.addEventListener('DOMContentLoaded', () => {
  // Check if Font Awesome is already loaded
  if (!document.querySelector('link[href*="fontawesome"]')) {
    const faLink = document.createElement('link');
    faLink.rel = 'stylesheet';
    faLink.href = 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css';
    document.head.appendChild(faLink);
  }

  // Check if the styles are already loaded
  if (!document.querySelector('link[href*="sidebar-chatbot.css"]') && !document.querySelector('#sidebar-chatbot-styles')) {
    const styleLink = document.createElement('link');
    styleLink.rel = 'stylesheet';
    styleLink.href = '/rag-chatbot/sidebar-chatbot.css';
    styleLink.id = 'sidebar-chatbot-styles';
    document.head.appendChild(styleLink);
  }

  // Initialize the chatbot widget after a small delay to ensure styles are loaded
  setTimeout(() => {
    new SidebarRAGChatbot();
  }, 100);
});

// Make the class available globally for custom initialization
window.SidebarRAGChatbot = SidebarRAGChatbot;