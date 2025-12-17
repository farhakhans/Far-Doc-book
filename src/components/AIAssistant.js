import React, { useState, useEffect } from 'react';
import clsx from 'clsx';

const AIAssistant = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. How can I help you today?", sender: 'ai' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleSendMessage = (e) => {
    e.preventDefault();
    if (inputValue.trim() === '') return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Simulate AI typing
    setIsTyping(true);

    // Simulate AI response after delay
    setTimeout(() => {
      const aiResponse = {
        id: Date.now() + 1,
        text: generateAIResponse(inputValue),
        sender: 'ai'
      };
      setMessages(prev => [...prev, aiResponse]);
      setIsTyping(false);
    }, 1000);
  };

  const generateAIResponse = (userInput) => {
    const input = userInput.toLowerCase();

    if (input.includes('hello') || input.includes('hi') || input.includes('hey')) {
      return "Hello there! I'm here to help you navigate the Physical AI & Humanoid Robotics book. You can ask me about any topic in the book, and I'll point you in the right direction.";
    } else if (input.includes('module') || input.includes('learn') || input.includes('start')) {
      return "Great! The book is organized into 4 main modules:\n1. [ROS 2 Fundamentals](/docs/modules/module-1-ros2/index)\n2. [Simulation Environments](/docs/modules/module-2-simulation/index)\n3. [NVIDIA Isaac Platform](/docs/modules/module-3-nvidia-isaac/index)\n4. [Vision-Language-Action Systems](/docs/modules/module-4-vla/index)\n\nWhich module would you like to know more about?";
    } else if (input.includes('ros') || input.includes('robot operating system')) {
      return "For ROS 2 content, check out [Module 1](/docs/modules/module-1-ros2/index). It covers nodes, topics, services, rclpy, and URDF. You can find practical exercises in the [practical-exercises](/docs/modules/module-1-ros2/practical-exercises) section of Module 1.";
    } else if (input.includes('simulation') || input.includes('gazebo') || input.includes('unity')) {
      return "For simulation environments, check out [Module 2](/docs/modules/module-2-simulation/index) which covers Gazebo and Unity. You'll find practical exercises and physics simulation concepts there.";
    } else if (input.includes('nvidia') || input.includes('isaac')) {
      return "For NVIDIA Isaac content, look at [Module 3](/docs/modules/module-3-nvidia-isaac/index) which covers Isaac Sim, Isaac ROS, and Nav2. The [practical exercises](/docs/modules/module-3-nvidia-isaac/practical-exercises) will help you get hands-on experience.";
    } else if (input.includes('vision') || input.includes('language') || input.includes('action') || input.includes('vla')) {
      return "For Vision-Language-Action systems, see [Module 4](/docs/modules/module-4-vla/index). This module covers integrating perception, language understanding, and robot actions using Whisper and LLMs.";
    } else if (input.includes('practical') || input.includes('exercise') || input.includes('project')) {
      return "Practical exercises are available in each module:\n- [Module 1 Exercises](/docs/modules/module-1-ros2/practical-exercises)\n- [Module 2 Exercises](/docs/modules/module-2-simulation/practical-exercises)\n- [Module 3 Exercises](/docs/modules/module-3-nvidia-isaac/practical-exercises)\n- [Module 4 Exercises](/docs/modules/module-4-vla/practical-exercises)\n\nThese provide hands-on experience with the concepts.";
    } else if (input.includes('hardware') || input.includes('requirements')) {
      return "For hardware requirements, check the [Hardware Requirements](/docs/hardware-requirements/index) section in the book. It covers different kit options including workstations, edge kits, and economy options.";
    } else if (input.includes('week') || input.includes('schedule') || input.includes('timeline')) {
      return "The book follows a 13-week academic quarter schedule. Check the [Weekly Breakdown](/docs/weekly-breakdown/week-1) section for week-by-week content and learning objectives.";
    } else if (input.includes('introduction') || input.includes('overview') || input.includes('intro')) {
      return "For an introduction to Physical AI & Humanoid Robotics, start with the [Introduction](/docs/intro) section. You can also check the [Overview](/docs/intro/overview) and [Learning Outcomes](/docs/intro/learning-outcomes).";
    } else if (input.includes('thank') || input.includes('thanks')) {
      return "You're welcome! Feel free to ask if you have more questions about Physical AI & Humanoid Robotics.";
    } else if (input.includes('help') || input.includes('assist')) {
      return "I can help you find information about:\n- [Book modules](/docs/modules/index)\n- [Practical exercises](/docs/category/modules#practical-exercises)\n- [Hardware requirements](/docs/hardware-requirements/index)\n- [Weekly breakdown](/docs/weekly-breakdown/week-1)\n- [Learning outcomes](/docs/intro/learning-outcomes)\n\nWhat would you like to know?";
    } else if (input.includes('ai assistant') || input.includes('you')) {
      return "I'm your AI assistant for the Physical AI & Humanoid Robotics book. I'm here to help you navigate the content and find information quickly. You can ask me about any topic in the book, and I'll point you to the relevant sections!";
    } else {
      return "I'm here to help with the Physical AI & Humanoid Robotics book. You can ask me about specific topics like [ROS 2](/docs/modules/module-1-ros2/index), [simulation](/docs/modules/module-2-simulation/index), [NVIDIA Isaac](/docs/modules/module-3-nvidia-isaac/index), [Vision-Language-Action systems](/docs/modules/module-4-vla/index), [practical exercises](/docs/category/modules#practical-exercises), or [hardware requirements](/docs/hardware-requirements/index). What would you like to know?";
    }
  };

  return (
    <div className={clsx('ai-assistant', { 'ai-assistant--open': isOpen })}>
      {isOpen ? (
        <div className="ai-assistant__container">
          <div className="ai-assistant__header">
            <div className="ai-assistant__title">Physical AI Assistant</div>
            <button
              className="ai-assistant__close"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="ai-assistant__messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  'ai-assistant__message',
                  `ai-assistant__message--${message.sender}`
                )}
              >
                <div className="ai-assistant__message-text">
                  {message.text.split('\n').map((line, i) => {
                    // Split line by markdown links
                    const parts = line.split(/(\[([^\]]+)\]\([^)]+\))/g);

                    return (
                      <div key={i}>
                        {parts.map((part, j) => {
                          // Check if this part is a markdown link
                          const linkMatch = part.match(/\[([^\]]+)\]\(([^)]+)\)/);
                          if (linkMatch) {
                            return (
                              <a
                                key={j}
                                href={linkMatch[2]}
                                className="ai-assistant__link"
                                target="_blank"
                                rel="noopener noreferrer"
                              >
                                {linkMatch[1]}
                              </a>
                            );
                          }
                          return <span key={j}>{part}</span>;
                        })}
                      </div>
                    );
                  })}
                </div>
              </div>
            ))}

            {isTyping && (
              <div className="ai-assistant__message ai-assistant__message--ai">
                <div className="ai-assistant__message-text ai-assistant__typing">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
          </div>

          <form className="ai-assistant__input-form" onSubmit={handleSendMessage}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask about Physical AI & Robotics..."
              className="ai-assistant__input"
            />
            <button type="submit" className="ai-assistant__send-button">
              Send
            </button>
          </form>
        </div>
      ) : null}

      <button
        className="ai-assistant__trigger"
        onClick={toggleChat}
        aria-label="Open AI assistant"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className="ai-assistant__icon"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </button>
    </div>
  );
};

export default AIAssistant;