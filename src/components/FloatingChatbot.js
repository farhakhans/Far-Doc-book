import React, { useState, useEffect, useRef } from 'react';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  // Format message text to handle newlines and make it more readable
  const formatMessageText = (text) => {
    return text.split('\n').map((line, i) => (
      <span key={i}>
        {line}
        <br />
      </span>
    ));
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Initialize with a welcome message
  useEffect(() => {
    if (messages.length === 0 && isOpen) {
      setMessages([{
        id: Date.now(),
        text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics documentation. Ask me anything about ROS 2, simulation, NVIDIA Isaac, VLA systems, or any other topics in the documentation.",
        sender: 'ai'
      }]);
    }
  }, [isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // Remove welcome message if it exists
      if (messages.length === 0) {
        setMessages([{
          id: Date.now(),
          text: "Welcome to AI Assistant! Ask me anything about the documentation.",
          sender: 'ai'
        }]);
      }
    }
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Handle common conversational inputs before sending to RAG system
    const lowerInput = inputValue.toLowerCase().trim();

    // Define greetings and common conversational phrases
    const greetings = ['hi', 'hello', 'hey', 'greetings', 'good morning', 'good afternoon', 'good evening'];
    const simpleQuestions = ['how are you', 'how do you do', 'what are you', 'who are you', 'what is this', 'what does this do'];
    const thanks = ['thank you', 'thanks', 'thank you very much', 'thanks a lot'];

    // Check if it's a greeting or conversational input
    if (greetings.some(greeting => lowerInput.includes(greeting))) {
      // Handle greetings
      const userMessage = {
        id: Date.now(),
        text: inputValue,
        sender: 'user'
      };
      setMessages(prev => [...prev, userMessage]);
      setInputValue('');

      // Send a friendly response
      const aiMessage = {
        id: Date.now() + 1,
        text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics documentation. How can I help you today? You can ask me about ROS 2, simulation, NVIDIA Isaac, VLA systems, or any other topics in the documentation.",
        sender: 'ai'
      };
      setMessages(prev => [...prev, aiMessage]);
      return;
    } else if (thanks.some(thank => lowerInput.includes(thank))) {
      // Handle thanks
      const userMessage = {
        id: Date.now(),
        text: inputValue,
        sender: 'user'
      };
      setMessages(prev => [...prev, userMessage]);
      setInputValue('');

      // Send a response to thanks
      const aiMessage = {
        id: Date.now() + 1,
        text: "You're welcome! Is there anything else I can help you with regarding the documentation?",
        sender: 'ai'
      };
      setMessages(prev => [...prev, aiMessage]);
      return;
    }

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Use MCP adapter to execute the ask_documentation tool
      const response = await fetch('http://localhost:5010/execute', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          tool_name: 'ask_documentation',
          arguments: {
            query: inputValue,
            context_size: 3
          }
        })
      });

      const data = await response.json();

      if (response.ok && data.success) {
        const aiMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'ai',
          contextSources: data.context_sources
        };
        setMessages(prev => [...prev, aiMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${data.error || data.message || 'Unknown error occurred'}`,
          sender: 'ai'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: `Sorry, I'm having trouble connecting to the documentation system. Please make sure the RAG services are running.`,
        sender: 'ai'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          boxShadow: '0 4px 15px rgba(102, 126, 234, 0.4)',
          fontSize: '24px',
          zIndex: '10000',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          transition: 'all 0.3s ease'
        }}
        aria-label="Open chat"
      >
        💬
      </button>

      {/* Chat Container */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '500px',
            background: 'white',
            borderRadius: '15px',
            boxShadow: '0 10px 40px rgba(0, 0, 0, 0.1)',
            display: 'flex',
            flexDirection: 'column',
            border: '1px solid #e1e4e5',
            zIndex: '9999'
          }}
        >
          {/* Header */}
          <div
            style={{
              background: 'rgba(255, 255, 255, 0.95)',
              padding: '1rem',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              borderBottom: '1px solid #e1e4e5'
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
              <div
                style={{
                  width: '30px',
                  height: '30px',
                  background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  color: 'white',
                  fontSize: '0.9rem'
                }}
              >
                🤖
              </div>
              <div>
                <h3 style={{ margin: 0, fontSize: '1.1rem', color: '#333' }}>AI Assistant</h3>
                <p style={{ margin: 0, fontSize: '0.8rem', color: '#666' }}>Documentation Helper</p>
              </div>
            </div>
            <button
              onClick={toggleChat}
              style={{
                background: '#f8f9fa',
                border: '1px solid #dee2e6',
                borderRadius: '50%',
                width: '30px',
                height: '30px',
                cursor: 'pointer',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '1.5rem',
              background: '#fafafa',
              display: 'flex',
              flexDirection: 'column',
              gap: '1rem'
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  alignSelf: message.sender === 'user' ? 'flex-end' : 'flex-start',
                  maxWidth: '85%',
                  marginBottom: '1rem'
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '1rem 1.25rem',
                    borderRadius: '18px',
                    fontSize: '1rem',
                    lineHeight: '1.5',
                    background: message.sender === 'user'
                      ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
                      : 'white',
                    color: message.sender === 'user' ? 'white' : '#333',
                    border: message.sender === 'user' ? 'none' : '1px solid #e1e4e5',
                    borderBottomRightRadius: message.sender === 'user' ? '5px' : '18px',
                    borderBottomLeftRadius: message.sender === 'user' ? '18px' : '5px',
                    boxShadow: message.sender === 'user' ? 'none' : '0 2px 10px rgba(0, 0, 0, 0.05)'
                  }}
                >
                  {formatMessageText(message.text)}
                </div>
                {/* Show context sources if available */}
                {message.contextSources && message.contextSources.length > 0 && (
                  <div
                    style={{
                      marginTop: '0.75rem',
                      padding: '0.75rem',
                      background: '#f8f9fa',
                      borderRadius: '8px',
                      fontSize: '0.85rem',
                      color: '#666',
                      borderLeft: '3px solid #667eea'
                    }}
                  >
                    <strong>Referenced documents:</strong><br />
                    {message.contextSources.map((ctx, idx) => (
                      <div key={idx} style={{ margin: '0.25rem 0' }}>
                        <em>{ctx.title}</em> ({ctx.filepath}) - Relevance: {(ctx.similarity * 100).toFixed(1)}%
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div
                style={{
                  alignSelf: 'flex-start',
                  maxWidth: '85%',
                  marginBottom: '1rem'
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '1rem 1.25rem',
                    background: 'white',
                    border: '1px solid #e1e4e5',
                    borderRadius: '18px',
                    borderBottomLeftRadius: '5px',
                    boxShadow: '0 2px 10px rgba(0, 0, 0, 0.05)'
                  }}
                >
                  <div style={{ display: 'flex', alignItems: 'center', gap: '0.25rem' }}>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      background: '#667eea',
                      borderRadius: '50%',
                      animation: 'typing 1.4s infinite ease-in-out'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      background: '#667eea',
                      borderRadius: '50%',
                      animation: 'typing 1.4s infinite ease-in-out',
                      animationDelay: '0.2s'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      background: '#667eea',
                      borderRadius: '50%',
                      animation: 'typing 1.4s infinite ease-in-out',
                      animationDelay: '0.4s'
                    }}></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div
            style={{
              padding: '1rem',
              background: 'white',
              borderTop: '1px solid #e1e4e5'
            }}
          >
            <div style={{ display: 'flex', gap: '0.5rem' }}>
              <button
                onClick={() => setMessages([])}
                title="New Chat"
                style={{
                  background: '#f8f9fa',
                  border: '1px solid #dee2e6',
                  borderRadius: '20px',
                  padding: '0.5rem 1rem',
                  cursor: 'pointer',
                  fontSize: '0.9rem',
                  whiteSpace: 'nowrap'
                }}
              >
                🔄 New
              </button>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about documentation..."
                style={{
                  flex: 1,
                  padding: '0.75rem 1rem',
                  border: '1px solid #e1e4e5',
                  borderRadius: '20px',
                  outline: 'none',
                  resize: 'none',
                  height: '45px',
                  fontSize: '0.9rem'
                }}
                rows={1}
              />
              <button
                onClick={sendMessage}
                disabled={isLoading}
                style={{
                  background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                  color: 'white',
                  border: 'none',
                  borderRadius: '50%',
                  width: '40px',
                  height: '40px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
              >
                ✉️
              </button>
            </div>
          </div>
        </div>
      )}

      <style jsx>{`
        @keyframes typing {
          0%, 60%, 100% { transform: translateY(0); }
          30% { transform: translateY(-5px); }
        }
      `}</style>
    </>
  );
};

export default FloatingChatbot;