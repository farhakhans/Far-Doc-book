// Frontend JavaScript for RAG Chatbot
class RAGChatbot {
  constructor() {
    this.mcpApiUrl = 'http://localhost:5010'; // MCP adapter URL
    this.chatContainer = document.getElementById('chat-messages');
    this.inputField = document.getElementById('user-input');
    this.sendButton = document.getElementById('send-button');
    this.loadingIndicator = document.getElementById('loading');
    this.clearChatButton = document.getElementById('clear-chat');

    // Remove welcome message when user starts typing
    this.inputField.addEventListener('focus', () => {
      this.removeWelcomeMessage();
    });

    this.initializeEventListeners();
  }

  initializeEventListeners() {
    this.sendButton.addEventListener('click', () => this.sendMessage());
    this.inputField.addEventListener('keypress', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        this.sendMessage();
      }
    });

    // Auto-resize textarea
    this.inputField.addEventListener('input', () => {
      this.autoResizeTextarea();
    });

    // Suggestion buttons
    document.querySelectorAll('.suggestion').forEach(button => {
      button.addEventListener('click', (e) => {
        const query = e.target.getAttribute('data-query') || e.target.closest('[data-query]').getAttribute('data-query');
        this.inputField.value = query;
        this.autoResizeTextarea();
        this.sendMessage();
      });
    });

    // Clear chat button
    if (this.clearChatButton) {
      this.clearChatButton.addEventListener('click', () => {
        if (confirm('Are you sure you want to clear the chat history?')) {
          this.clearChat();
        }
      });
    }
  }

  autoResizeTextarea() {
    this.inputField.style.height = 'auto';
    this.inputField.style.height = Math.min(this.inputField.scrollHeight, 120) + 'px';
  }

  removeWelcomeMessage() {
    const welcomeMessage = this.chatContainer.querySelector('.welcome-message');
    if (welcomeMessage) {
      welcomeMessage.remove();
    }
  }

  clearChat() {
    // Keep the welcome message
    const welcomeContent = this.chatContainer.innerHTML;
    this.chatContainer.innerHTML = '<div class="welcome-message">' +
      '<h2><i class="fas fa-graduation-cap"></i> Welcome to Docusaurus Documentation Chat</h2>' +
      '<p>I\'m your AI assistant for the Physical AI & Humanoid Robotics book. I can help you find information about:</p>' +
      '<div class="suggestions">' +
      '<div class="suggestion" data-query="What is ROS 2?">ROS 2 Fundamentals</div>' +
      '<div class="suggestion" data-query="Tell me about simulation environments">Simulation</div>' +
      '<div class="suggestion" data-query="Explain NVIDIA Isaac platform">Isaac Platform</div>' +
      '<div class="suggestion" data-query="What are Vision-Language-Action systems?">VLA Systems</div>' +
      '</div>' +
      '</div>';

    // Re-add event listeners to the new suggestion elements
    document.querySelectorAll('.suggestion').forEach(button => {
      button.addEventListener('click', (e) => {
        const query = e.target.getAttribute('data-query') || e.target.closest('[data-query]').getAttribute('data-query');
        this.inputField.value = query;
        this.autoResizeTextarea();
        this.sendMessage();
      });
    });
  }

  async sendMessage() {
    const message = this.inputField.value.trim();
    if (!message) return;

    // Remove welcome message if it's the first interaction
    this.removeWelcomeMessage();

    // Add user message to chat
    this.addMessage('user', message);
    this.inputField.value = '';
    this.inputField.style.height = '60px';
    this.showLoading(true);

    try {
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

      const data = await response.json();

      if (response.ok && data.success) {
        // Remove loading indicator
        this.showLoading(false);
        // Add AI response to chat with context sources
        this.addMessage('ai', data.response, data.context_sources);
      } else {
        this.showLoading(false);
        this.addMessage('ai', `Error: ${data.error || data.message || 'Unknown error occurred'}`);
      }
    } catch (error) {
      this.showLoading(false);
      this.addMessage('ai', `Connection error: ${error.message}`);
    }
  }

  addMessage(sender, text, contextSources = null) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `message message-${sender}`;

    // Create message header with icon
    const headerDiv = document.createElement('div');
    headerDiv.className = 'message-header';

    const iconDiv = document.createElement('div');
    iconDiv.className = `message-icon ${sender}-icon`;
    iconDiv.innerHTML = sender === 'user' ? '<i class="fas fa-user"></i>' : '<i class="fas fa-robot"></i>';

    const nameSpan = document.createElement('span');
    nameSpan.textContent = sender === 'user' ? 'You' : 'AI Assistant';

    headerDiv.appendChild(iconDiv);
    headerDiv.appendChild(nameSpan);

    // Create message content
    const contentDiv = document.createElement('div');
    contentDiv.className = 'message-content';
    contentDiv.innerHTML = this.formatMessage(text);

    messageDiv.appendChild(headerDiv);
    messageDiv.appendChild(contentDiv);

    // Add context sources if provided
    if (contextSources && contextSources.length > 0) {
      const contextDiv = document.createElement('div');
      contextDiv.className = 'message-context';
      contextDiv.innerHTML = '<strong>Referenced documents:</strong><br>' +
        contextSources.map(ctx =>
          `<div class="context-source"><em>${ctx.title}</em> (${ctx.filepath}) - Relevance: ${(ctx.similarity * 100).toFixed(1)}%</div>`
        ).join('');

      messageDiv.appendChild(contextDiv);
    }

    this.chatContainer.appendChild(messageDiv);
    this.scrollToBottom();
  }

  formatMessage(text) {
    // Simple formatting - convert newlines to <br> and preserve some markdown
    return text.replace(/\n/g, '<br>');
  }

  showLoading(show) {
    if (this.loadingIndicator) {
      this.loadingIndicator.style.display = show ? 'block' : 'none';

      if (show) {
        // Add loading message to chat if not already present
        const existingLoading = this.chatContainer.querySelector('.loading-message');
        if (!existingLoading) {
          const loadingDiv = document.createElement('div');
          loadingDiv.className = 'message message-ai loading-message';
          loadingDiv.innerHTML = `
            <div class="message-header">
              <div class="message-icon ai-icon"><i class="fas fa-robot"></i></div>
              <span>AI Assistant</span>
            </div>
            <div class="message-content">
              <div class="typing-indicator">
                <div class="typing-dots">
                  <div class="typing-dot"></div>
                  <div class="typing-dot"></div>
                  <div class="typing-dot"></div>
                </div>
              </div>
            </div>
          `;
          this.chatContainer.appendChild(loadingDiv);
          this.scrollToBottom();
        }
      } else {
        // Remove loading message
        const loadingMessage = this.chatContainer.querySelector('.loading-message');
        if (loadingMessage) {
          loadingMessage.remove();
        }
      }
    }
  }

  scrollToBottom() {
    this.chatContainer.scrollTop = this.chatContainer.scrollHeight;
  }
}

// Initialize chatbot when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  new RAGChatbot();
});