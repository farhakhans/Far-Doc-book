// Floating RAG Chatbot Widget
class FloatingRAGChatbot {
  constructor(options = {}) {
    this.mcpApiUrl = options.mcpApiUrl || 'http://localhost:5010'; // MCP adapter URL
    this.isOpen = false;
    this.initializeWidget();
    this.initializeEventListeners();
  }

  initializeWidget() {
    // Create the widget HTML
    const widgetHTML = `
      <!-- Chatbot Toggle Button -->
      <button id="chatbot-toggle" class="chatbot-toggle" style="
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 60px;
        height: 60px;
        border-radius: 50%;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        border: none;
        cursor: pointer;
        box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 24px;
        z-index: 10000;
        transition: all 0.3s ease;
      ">
        <i class="fas fa-robot"></i>
      </button>

      <!-- Chatbot Container (Initially Hidden) -->
      <div id="chatbot-container" class="chatbot-container" style="
        position: fixed;
        bottom: 90px;
        right: 20px;
        width: 400px;
        height: 500px;
        background: white;
        border-radius: 15px;
        box-shadow: 0 10px 40px rgba(0, 0, 0, 0.1);
        display: none;
        flex-direction: column;
        overflow: hidden;
        border: 1px solid #e1e4e5;
        z-index: 9999;
        transition: all 0.3s ease;
      ">
        <!-- Chatbot Header -->
        <div class="chatbot-header" style="
          background: rgba(255, 255, 255, 0.95);
          backdrop-filter: blur(10px);
          padding: 1rem;
          display: flex;
          align-items: center;
          justify-content: space-between;
          border-bottom: 1px solid #e1e4e5;
        ">
          <div style="display: flex; align-items: center; gap: 0.75rem;">
            <div style="
              width: 30px;
              height: 30px;
              background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
              border-radius: 50%;
              display: flex;
              align-items: center;
              justify-content: center;
              color: white;
              font-size: 0.9rem;
            ">
              <i class="fas fa-robot"></i>
            </div>
            <div>
              <h3 style="margin: 0; font-size: 1.1rem; color: #333;">AI Assistant</h3>
              <p style="margin: 0; font-size: 0.8rem; color: #666;">Documentation Helper</p>
            </div>
          </div>
          <button id="chatbot-close" class="chatbot-close" style="
            background: #f8f9fa;
            border: 1px solid #dee2e6;
            border-radius: 50%;
            width: 30px;
            height: 30px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
          ">
            <i class="fas fa-times"></i>
          </button>
        </div>

        <!-- Chat Messages Area -->
        <div id="chat-messages" class="chat-messages" style="
          flex: 1;
          overflow-y: auto;
          padding: 1.5rem;
          background: #fafafa;
          display: flex;
          flex-direction: column;
          gap: 1rem;
        ">
          <div class="welcome-message" style="text-align: center; color: #666; flex: 1; display: flex; flex-direction: column; justify-content: center;">
            <h4 style="color: #333; margin-bottom: 1rem;"><i class="fas fa-robot"></i> Welcome to AI Assistant</h4>
            <p>Ask me anything about the documentation!</p>
          </div>
        </div>

        <!-- Input Area -->
        <div class="chat-input-area" style="
          padding: 1rem;
          background: white;
          border-top: 1px solid #e1e4e5;
        ">
          <div style="display: flex; gap: 0.5rem;">
            <textarea
              id="user-input"
              class="user-input"
              placeholder="Ask about documentation..."
              style="
                flex: 1;
                padding: 0.75rem 1rem;
                border: 1px solid #e1e4e5;
                border-radius: 20px;
                outline: none;
                resize: none;
                height: 45px;
                max-height: 100px;
                font-size: 0.9rem;
              "
            ></textarea>
            <button id="send-button" class="send-button" style="
              background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
              color: white;
              border: none;
              border-radius: 50%;
              width: 40px;
              height: 40px;
              cursor: pointer;
              display: flex;
              align-items: center;
              justify-content: center;
            ">
              <i class="fas fa-paper-plane"></i>
            </button>
          </div>
        </div>
      </div>
    `;

    // Add the widget to the page
    document.body.insertAdjacentHTML('beforeend', widgetHTML);

    // Store references to elements
    this.toggleButton = document.getElementById('chatbot-toggle');
    this.container = document.getElementById('chatbot-container');
    this.closeButton = document.getElementById('chatbot-close');
    this.chatMessages = document.getElementById('chat-messages');
    this.userInput = document.getElementById('user-input');
    this.sendButton = document.getElementById('send-button');
  }

  initializeEventListeners() {
    // Toggle chatbot visibility
    this.toggleButton.addEventListener('click', () => this.toggleChatbot());
    this.closeButton.addEventListener('click', () => this.closeChatbot());

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
    this.container.style.display = this.isOpen ? 'flex' : 'none';
    if (this.isOpen) {
      // Remove welcome message when opening
      const welcomeMessage = this.chatMessages.querySelector('.welcome-message');
      if (welcomeMessage) {
        welcomeMessage.remove();
      }
      this.userInput.focus();
    }
  }

  closeChatbot() {
    this.isOpen = false;
    this.container.style.display = 'none';
  }

  async sendMessage() {
    const message = this.userInput.value.trim();
    if (!message) return;

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

  showTypingIndicator() {
    const typingDiv = document.createElement('div');
    typingDiv.className = 'message message-ai typing-indicator';
    typingDiv.style.cssText = `
      align-self: flex-start;
      max-width: 85%;
      margin-bottom: 1rem;
    `;

    typingDiv.innerHTML = `
      <div style="
        display: inline-block;
        padding: 1rem 1.25rem;
        background: white;
        border: 1px solid #e1e4e5;
        border-radius: 18px;
        border-bottom-left-radius: 5px;
        box-shadow: 0 2px 10px rgba(0, 0, 0, 0.05);
      ">
        <div style="display: flex; align-items: center; gap: 0.25rem;">
          <div style="
            width: 8px;
            height: 8px;
            background: #667eea;
            border-radius: 50%;
            animation: typing 1.4s infinite ease-in-out;
          "></div>
          <div style="
            width: 8px;
            height: 8px;
            background: #667eea;
            border-radius: 50%;
            animation: typing 1.4s infinite ease-in-out;
            animation-delay: 0.2s;
          "></div>
          <div style="
            width: 8px;
            height: 8px;
            background: #667eea;
            border-radius: 50%;
            animation: typing 1.4s infinite ease-in-out;
            animation-delay: 0.4s;
          "></div>
        </div>
      </div>
    `;

    // Add CSS animation if not already present
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
    messageDiv.className = `message message-${sender}`;
    messageDiv.style.cssText = `
      align-self: ${sender === 'user' ? 'flex-end' : 'flex-start'};
      max-width: 85%;
      margin-bottom: 1rem;
    `;

    // Create message content
    const contentDiv = document.createElement('div');
    contentDiv.style.cssText = `
      display: inline-block;
      padding: 1rem 1.25rem;
      border-radius: 18px;
      font-size: 1rem;
      line-height: 1.5;
      background: ${sender === 'user'
        ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
        : 'white'};
      color: ${sender === 'user' ? 'white' : '#333'};
      border: ${sender === 'user' ? 'none' : '1px solid #e1e4e5'};
      border-${sender === 'user' ? 'bottom-right' : 'bottom-left'}-radius: 5px;
      box-shadow: ${sender === 'user' ? 'none' : '0 2px 10px rgba(0, 0, 0, 0.05)'};
    `;

    // Format and add the message text
    contentDiv.innerHTML = this.formatMessage(text);

    messageDiv.appendChild(contentDiv);

    // Add context sources if provided
    if (contextSources && contextSources.length > 0) {
      const contextDiv = document.createElement('div');
      contextDiv.style.cssText = `
        margin-top: 0.75rem;
        padding: 0.75rem;
        background: #f8f9fa;
        border-radius: 8px;
        font-size: 0.85rem;
        color: #666;
        border-left: 3px solid #667eea;
      `;

      contextDiv.innerHTML = '<strong>Referenced documents:</strong><br>' +
        contextSources.map(ctx =>
          `<div style="margin: 0.25rem 0;"><em>${ctx.title}</em> (${ctx.filepath}) - Relevance: ${(ctx.similarity * 100).toFixed(1)}%</div>`
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

// Ensure the widget initializes after the page is fully loaded
function initializeChatbot() {
  // Check if Font Awesome is already loaded
  if (!document.querySelector('link[href*="fontawesome"]')) {
    const faLink = document.createElement('link');
    faLink.rel = 'stylesheet';
    faLink.href = 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css';
    document.head.appendChild(faLink);
  }

  // Initialize the chatbot widget
  new FloatingRAGChatbot();
}

// Try multiple methods to ensure the chatbot initializes
if (document.readyState === 'loading') {
  // Document is still loading, wait for DOMContentLoaded
  document.addEventListener('DOMContentLoaded', initializeChatbot);
} else {
  // Document is already loaded, initialize immediately
  setTimeout(initializeChatbot, 100);
}

// Also try to initialize when the window loads
window.addEventListener('load', initializeChatbot);

// As a fallback, try to initialize after a delay
setTimeout(() => {
  if (!window.floatingChatbotInitialized) {
    initializeChatbot();
    window.floatingChatbotInitialized = true;
  }
}, 1000);

// Make the class available globally for custom initialization
window.FloatingRAGChatbot = FloatingRAGChatbot;