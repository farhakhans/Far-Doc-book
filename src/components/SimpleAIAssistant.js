import React, { useState, useRef, useEffect } from 'react';

const SimpleAIAssistant = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hi! I'm Farha, your AI assistant for Physical AI & Robotics. How can I help you today?", sender: 'ai' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    // Use setTimeout with a slight delay to ensure DOM is fully updated
    setTimeout(() => {
      const element = messagesEndRef.current;
      if (element) {
        element.scrollIntoView({ behavior: 'smooth', block: 'end' });
      }
    }, 50); // Small delay to ensure DOM update
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

    // Scroll to bottom after adding user message with slight delay
    setTimeout(() => scrollToBottom(), 50);

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

      // Scroll to bottom after adding AI response with slight delay
      setTimeout(() => scrollToBottom(), 50);
    }, 1000);
  };

  // Function to render messages with markdown-style links
  const renderMessageWithLinks = (text) => {
    // Split text by markdown links [text](url)
    const parts = text.split(/(\[[^\]]+\]\([^)]+\))/g);

    return parts.map((currentPart, index) => {
      // Check if this part is a markdown link
      const linkMatch = currentPart.match(/\[([^\]]+)\]\(([^)]+)\)/);

      if (linkMatch) {
        // This is a markdown link
        const [, linkText, linkUrl] = linkMatch;

        return (
          <a
            key={index}
            href={linkUrl}
            style={{
              color: '#0d9488', // Teal link color
              textDecoration: 'underline',
              fontWeight: '500',
              cursor: 'pointer'
            }}
            onClick={(e) => {
              e.preventDefault();
              // Open the link in a new tab
              window.open(linkUrl, '_blank');
            }}
          >
            {linkText}
          </a>
        );
      } else {
        // This is plain text, split by newlines
        return currentPart.split('\n').map((line, lineIndex) => (
          <div key={`${index}-${lineIndex}`}>
            {line.split('**').map((segment, segIndex, arr) => {
              // Handle bold text surrounded by **
              if (segIndex % 2 === 1) {
                // Odd indices are the bold text
                return <strong key={`${index}-${lineIndex}-${segIndex}`} style={{fontWeight: '600'}}>{segment}</strong>;
              } else if (segIndex < arr.length - 1) {
                // Even indices are the normal text before bold
                return segment;
              } else {
                return segment;
              }
            })}
          </div>
        ));
      }
    });
  };

  const generateAIResponse = (userInput) => {
    const input = userInput.toLowerCase();

    // Enhanced response generation with more specific knowledge
    if (input.includes('hello') || input.includes('hi') || input.includes('hey')) {
      return "Hello there! I'm Farha, your AI assistant for the Physical AI & Humanoid Robotics book. I can help you find specific topics, explain concepts, or guide you through the learning materials. What would you like to explore today?";
    } else if (input.includes('module') || input.includes('learn') || input.includes('start')) {
      return "The book is organized into 4 comprehensive modules:\n1. [ROS 2 Fundamentals](/docs/modules/module-1-ros2/index) - Nodes, topics, services, rclpy, URDF\n2. [Simulation Environments](/docs/modules/module-2-simulation/index) - Gazebo, Unity, physics simulation\n3. [NVIDIA Isaac Platform](/docs/modules/module-3-nvidia-isaac/index) - Isaac Sim, Isaac ROS, Nav2\n4. [Vision-Language-Action Systems](/docs/modules/module-4-vla/index) - Whisper, LLMs, VLA integration\n\nWhich module interests you most?";
    } else if (input.includes('ros') || input.includes('robot operating system')) {
      return "For ROS 2 fundamentals, check out [Module 1](/docs/modules/module-1-ros2/index). This covers:\n- Core concepts: nodes, topics, services\n- Programming: rclpy for Python\n- Robot description: URDF\n- Practical exercises: [Module 1 Exercises](/docs/modules/module-1-ros2/practical-exercises)\n\nKey concepts include distributed computing, real-time support, and Quality of Service (QoS) policies.";
    } else if (input.includes('simulation') || input.includes('gazebo') || input.includes('unity')) {
      return "For simulation environments, see [Module 2](/docs/modules/module-2-simulation/index). This includes:\n- Gazebo: Physics simulation and sensor modeling\n- Unity: Robotics simulation with perception\n- Physics simulation: Collision detection, dynamics\n- Practical exercises: [Module 2 Exercises](/docs/modules/module-2-simulation/practical-exercises)\n\nThese tools let you test robot behaviors safely before real-world deployment.";
    } else if (input.includes('nvidia') || input.includes('isaac') || input.includes('nav2')) {
      return "For NVIDIA Isaac tools, see [Module 3](/docs/modules/module-3-nvidia-isaac/index). This covers:\n- Isaac Sim: Advanced simulation environment\n- Isaac ROS: GPU-accelerated perception and navigation\n- Nav2: Navigation system for autonomous robots\n- Practical exercises: [Module 3 Exercises](/docs/modules/module-3-nvidia-isaac/practical-exercises)\n\nThese tools provide GPU acceleration for complex robotics tasks.";
    } else if (input.includes('vision') || input.includes('language') || input.includes('action') || input.includes('vla') || input.includes('whisper') || input.includes('llm')) {
      return "For Vision-Language-Action systems, see [Module 4](/docs/modules/module-4-vla/index). This includes:\n- Voice processing: OpenAI Whisper for speech recognition\n- Language models: LLMs for understanding commands\n- Action generation: Converting language to robot actions\n- Practical exercises: [Module 4 Exercises](/docs/modules/module-4-vla/practical-exercises)\n\nThis creates complete AI systems that can understand spoken commands and act upon them.";
    } else if (input.includes('nodes') || input.includes('topics') || input.includes('services')) {
      return "In ROS 2 (covered in [Module 1](/docs/modules/module-1-ros2/index)):\n- **Nodes**: Independent processes that perform computation\n- **Topics**: Publish/subscribe communication pattern\n- **Services**: Request/response communication pattern\n\nThese form the foundation of ROS 2's distributed architecture. See the [Nodes, Topics, Services](/docs/modules/module-1-ros2/nodes-topics-services) section for details.";
    } else if (input.includes('rclpy') || input.includes('python')) {
      return "rclpy is the Python client library for ROS 2. In [Module 1](/docs/modules/module-1-ros2/index), you'll find:\n- How to create publishers and subscribers in Python\n- Parameter management\n- Timer usage\n- Lifecycle nodes\n\nSee the [rclpy section](/docs/modules/module-1-ros2/rclpy) for hands-on examples.";
    } else if (input.includes('urdf') || input.includes('robot model')) {
      return "URDF (Unified Robot Description Format) is used to describe robot models in ROS 2. In [Module 1](/docs/modules/module-1-ros2/index), the [URDF section](/docs/modules/module-1-ros2/urdf) covers:\n- Defining robot kinematics\n- Joint types and limits\n- Visual and collision properties\n- Robot state publishing\n\nThis is essential for simulation and visualization.";
    } else if (input.includes('practical') || input.includes('exercise') || input.includes('project')) {
      return "Practical exercises are available in each module:\n- [Module 1 Exercises](/docs/modules/module-1-ros2/practical-exercises): ROS 2 basics\n- [Module 2 Exercises](/docs/modules/module-2-simulation/practical-exercises): Simulation environments\n- [Module 3 Exercises](/docs/modules/module-3-nvidia-isaac/practical-exercises): Isaac platform\n- [Module 4 Exercises](/docs/modules/module-4-vla/practical-exercises): Vision-Language-Action systems\n\nEach includes step-by-step instructions and solution templates.";
    } else if (input.includes('hardware') || input.includes('requirements') || input.includes('kit')) {
      return "For hardware requirements, see the [Hardware Requirements section](/docs/hardware-requirements/index):\n- [Workstations](/docs/hardware-requirements/workstations): High-performance computing options\n- [Edge Kit](/docs/hardware-requirements/edge-kit): Compact, portable solutions\n- [Economy Kit](/docs/hardware-requirements/economy-kit): Budget-friendly options\n- [Robot Lab Options](/docs/hardware-requirements/robot-lab-options): Multiple robot setups\n\nChoose based on your budget and learning objectives.";
    } else if (input.includes('week') || input.includes('schedule') || input.includes('timeline')) {
      return "The book follows a 13-week academic quarter schedule:\n- Weeks 1-3: [ROS 2 fundamentals](/docs/weekly-breakdown/week-1)\n- Weeks 4-5: [Simulation environments](/docs/weekly-breakdown/week-4)\n- Weeks 6-9: [NVIDIA Isaac platform](/docs/weekly-breakdown/week-6)\n- Weeks 10-12: [Vision-Language-Action systems](/docs/weekly-breakdown/week-10)\n- Week 13: [Capstone project](/docs/weekly-breakdown/week-13)\n\nSee each week's breakdown for specific learning objectives and assignments.";
    } else if (input.includes('learning') || input.includes('outcome') || input.includes('objective')) {
      return "The learning outcomes are detailed in the [Learning Outcomes section](/docs/intro/learning-outcomes):\n- Understand Physical AI fundamentals and embodied intelligence\n- Implement ROS 2 communication patterns\n- Design simulation-based testing environments\n- Utilize NVIDIA Isaac platform tools\n- Implement Vision-Language-Action systems\n- Apply AI to embodied systems like humanoid robots\n\nThese outcomes guide your learning journey through the book.";
    } else if (input.includes('physical ai') || input.includes('embodied intelligence')) {
      return "Physical AI represents a paradigm shift from traditional digital AI to AI that operates in the physical world. Key concepts include:\n- Embodied intelligence: Intelligence emerges from agent-environment interaction\n- Physics and kinematics understanding\n- Sensorimotor coordination\n- Real-time decision making\n- Adaptation to environmental changes\n- Safe interaction with humans and objects\n\nSee the [Introduction](/docs/intro) for foundational concepts.";
    } else if (input.includes('thank') || input.includes('thanks')) {
      return "You're very welcome! I'm glad I could help. Feel free to ask if you have more questions about Physical AI & Humanoid Robotics.";
    } else if (input.includes('help') || input.includes('assist') || input.includes('guide')) {
      return "I can help with:\n- Finding specific modules ([Modules Overview](/docs/modules/index))\n- Explaining ROS 2 concepts ([Module 1](/docs/modules/module-1-ros2/index))\n- Simulation environments ([Module 2](/docs/modules/module-2-simulation/index))\n- NVIDIA Isaac tools ([Module 3](/docs/modules/module-3-nvidia-isaac/index))\n- Vision-Language-Action systems ([Module 4](/docs/modules/module-4-vla/index))\n- Hardware requirements ([Hardware Section](/docs/hardware-requirements/index))\n- Learning outcomes ([Learning Outcomes](/docs/intro/learning-outcomes))\n\nWhat specific topic would you like to explore?";
    } else if (input.includes('introduction') || input.includes('overview')) {
      return "The introduction section covers foundational concepts of Physical AI. Check out the [Introduction](/docs/intro) and [Overview](/docs/intro/overview) for:\n- What is Physical AI?\n- Embodied intelligence concepts\n- Why Physical AI matters\n- How the book is structured\n\nThis provides essential context for the rest of your learning journey.";
    } else if (input.includes('assessment') || input.includes('project') || input.includes('evaluation')) {
      return "Assessment projects are designed to validate your learning:\n- [ROS 2 Project](/docs/assessments/ros2-project): Implement ROS 2 communication patterns\n- [Simulation Project](/docs/assessments/simulation-project): Create simulation environments\n- [Isaac Project](/docs/assessments/isaac-project): Use NVIDIA Isaac tools\n- [Capstone Project](/docs/assessments/capstone-project): Integrate all concepts\n\nEach project includes detailed requirements and evaluation rubrics.";
    } else if (input.includes('kinematics') || input.includes('dynamics')) {
      return "Kinematics and dynamics are fundamental to robotics:\n- **Kinematics**: Study of motion without considering forces (forward and inverse kinematics)\n- **Dynamics**: Study of motion with forces and torques\n- These concepts are essential for robot control and simulation\n- See [Module 1](/docs/modules/module-1-ros2/index) for foundational concepts and [Module 2](/docs/modules/module-2-simulation/index) for simulation of these concepts.";
    } else if (input.includes('computer vision') || input.includes('cv') || input.includes('perception')) {
      return "Computer vision and perception are key components of robotics:\n- **Computer Vision**: Algorithms for image processing and understanding\n- **Sensor Fusion**: Combining data from multiple sensors\n- **Perception Systems**: How robots understand their environment\n- Covered in [Module 2](/docs/modules/module-2-simulation/index) for simulation and [Module 3](/docs/modules/module-3-nvidia-isaac/index) for NVIDIA Isaac tools.";
    } else if (input.includes('path planning') || input.includes('motion planning') || input.includes('trajectory')) {
      return "Path and motion planning are crucial for autonomous robots:\n- **Path Planning**: Finding routes from start to goal\n- **Motion Planning**: Planning detailed movements considering dynamics\n- **Trajectory Generation**: Creating time-parameterized paths\n- See [Module 3](/docs/modules/module-3-nvidia-isaac/index) for Nav2 navigation system and [Module 2](/docs/modules/module-2-simulation/index) for simulation.";
    } else if (input.includes('control theory') || input.includes('pid') || input.includes('controllers')) {
      return "Control theory is essential for robot behavior:\n- **PID Controllers**: Proportional-Integral-Derivative control\n- **Feedback Control**: Adjusting based on sensor measurements\n- **System Dynamics**: Understanding how systems respond to inputs\n- These concepts are applied throughout the book, especially in [Module 1](/docs/modules/module-1-ros2/index) and [Module 2](/docs/modules/module-2-simulation/index).";
    } else if (input.includes('machine learning') || input.includes('ml') || input.includes('reinforcement learning') || input.includes('rl')) {
      return "Machine learning is increasingly important in robotics:\n- **Supervised Learning**: Training models with labeled data\n- **Reinforcement Learning**: Learning through interaction with environment\n- **Deep Learning**: Neural networks for complex pattern recognition\n- See [Module 4](/docs/modules/module-4-vla/index) for Vision-Language-Action systems that incorporate ML techniques.";
    } else if (input.includes('ethics') || input.includes('safety') || input.includes('human robot interaction') || input.includes('hri')) {
      return "Robot ethics and safety are critical for physical AI:\n- **Safety**: Ensuring robots operate without harm\n- **Ethics**: Moral considerations in AI and robotics\n- **Human-Robot Interaction**: How humans and robots work together\n- These topics are integrated throughout the book's approach to Physical AI and embodied systems.";
    } else {
      // For more general queries, try to identify keywords and provide relevant information
      const generalResponse = [];

      if (input.includes('robot')) {
        generalResponse.push("- **Robotics**: The book covers various aspects of robotics including kinematics, dynamics, and control systems");
      }

      if (input.includes('ai') || input.includes('artificial intelligence')) {
        generalResponse.push("- **Artificial Intelligence**: Focuses on Physical AI, which combines AI with physical systems");
      }

      if (input.includes('humanoid')) {
        generalResponse.push("- **Humanoid Robots**: Covered in the context of embodied intelligence and physical AI");
      }

      if (input.includes('simulation')) {
        generalResponse.push("- **Simulation**: Check out [Module 2](/docs/modules/module-2-simulation/index) for Gazebo and Unity simulation environments");
      }

      if (input.includes('python')) {
        generalResponse.push("- **Python Programming**: Used extensively with rclpy for ROS 2 development");
      }

      if (input.includes('control') || input.includes('motion')) {
        generalResponse.push("- **Robot Control**: Learn about motion planning and control systems in the ROS 2 module");
      }

      if (input.includes('sensor') || input.includes('perception')) {
        generalResponse.push("- **Sensors & Perception**: Covered in simulation and NVIDIA Isaac modules");
      }

      if (input.includes('navigation') || input.includes('path')) {
        generalResponse.push("- **Navigation**: Learn about Nav2 in [Module 3](/docs/modules/module-3-nvidia-isaac/index)");
      }

      if (input.includes('deep learning') || input.includes('neural')) {
        generalResponse.push("- **Deep Learning**: Applied in Vision-Language-Action systems in [Module 4](/docs/modules/module-4-vla/index)");
      }

      if (input.includes('kinematics') || input.includes('dynamics')) {
        generalResponse.push("- **Kinematics & Dynamics**: Covered in the context of robot modeling and control");
      }

      if (input.includes('computer vision') || input.includes('cv')) {
        generalResponse.push("- **Computer Vision**: Part of perception systems in simulation and NVIDIA Isaac modules");
      }

      if (input.includes('path planning') || input.includes('motion planning')) {
        generalResponse.push("- **Path & Motion Planning**: Covered in navigation systems and robotics control");
      }

      if (input.includes('control theory') || input.includes('pid')) {
        generalResponse.push("- **Control Theory**: Essential for robot control systems");
      }

      if (input.includes('machine learning') || input.includes('ml')) {
        generalResponse.push("- **Machine Learning**: Applied throughout the book, especially in Vision-Language-Action systems");
      }

      if (input.includes('reinforcement learning') || input.includes('rl')) {
        generalResponse.push("- **Reinforcement Learning**: Relevant to autonomous robot behavior in simulation environments");
      }

      if (input.includes('ethics') || input.includes('safety')) {
        generalResponse.push("- **Robot Ethics & Safety**: Important considerations for physical AI systems");
      }

      if (input.includes('multi agent') || input.includes('swarm')) {
        generalResponse.push("- **Multi-Agent Systems**: Relevant to coordination of multiple robots");
      }

      if (input.includes('real time') || input.includes('rtos')) {
        generalResponse.push("- **Real-Time Systems**: Important for robotics applications, covered in ROS 2 fundamentals");
      }

      if (generalResponse.length > 0) {
        return `Based on your question about "${userInput}", here's relevant information:\n\n${generalResponse.join('\n')}\n\nWould you like more specific details about any of these topics?`;
      } else {
        return "I'm here to help with the Physical AI & Humanoid Robotics book. You can ask me about specific topics like:\n- ROS 2 fundamentals (nodes, topics, services, rclpy, URDF)\n- Simulation environments (Gazebo, Unity)\n- NVIDIA Isaac tools (Isaac Sim, Isaac ROS, Nav2)\n- Vision-Language-Action systems (Whisper, LLMs)\n- Hardware requirements and kit options\n- Weekly breakdown and learning objectives\n\nWhat would you like to know more about?";
      }
    }
  };

  return (
    <div
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000,
      }}
    >
      {!isOpen ? (
        <button
          onClick={() => setIsOpen(true)}
          style={{
            backgroundColor: '#14b8a6', // Teal color
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '60px',
            height: '60px',
            fontSize: '20px', // Reduced font size
            cursor: 'pointer',
            zIndex: 1000,
            boxShadow: '0 4px 20px rgba(20, 184, 166, 0.4)', // Teal shadow
            transition: 'all 0.3s ease'
          }}
          onMouseOver={(e) => {
            e.target.style.transform = 'scale(1.1)';
            e.target.style.boxShadow = '0 6px 25px rgba(20, 184, 166, 0.6)';
          }}
          onMouseOut={(e) => {
            e.target.style.transform = 'scale(1)';
            e.target.style.boxShadow = '0 4px 20px rgba(20, 184, 166, 0.4)';
          }}
        >
          💬
        </button>
      ) : (
        <div
          style={{
            width: '350px',
            height: '500px',
            backgroundColor: '#ffffff',
            borderRadius: '16px',
            boxShadow: '0 10px 40px rgba(0,0,0,0.15)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
          }}
        >
          <div
            style={{
              backgroundColor: '#14b8a6', // Teal header
              color: 'white',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <div style={{fontWeight: '600', fontSize: '16px'}}>Farha AI chatbot</div>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={() => setIsOpen(false)}
                title="Minimize"
                style={{
                  background: 'rgba(255,255,255,0.2)',
                  border: 'none',
                  color: 'white',
                  fontSize: '16px',
                  cursor: 'pointer',
                  width: '30px',
                  height: '30px',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
              >
                −
              </button>
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  setIsOpen(false);
                }}
                title="Close"
                style={{
                  background: 'rgba(255,255,255,0.2)',
                  border: 'none',
                  color: 'white',
                  fontSize: '20px',
                  cursor: 'pointer',
                  width: '30px',
                  height: '30px',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
              >
                ×
              </button>
            </div>
          </div>

          <div
            ref={messagesEndRef}
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              backgroundColor: '#f0fdfa',
              display: 'flex',
              flexDirection: 'column'
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  marginBottom: '12px',
                  textAlign: message.sender === 'user' ? 'right' : 'left',
                  display: 'flex',
                  justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start'
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '10px 14px', // Reduced padding
                    backgroundColor: message.sender === 'user' ? '#14b8a6' : '#d1fae5', // Teal colors
                    color: message.sender === 'user' ? 'white' : '#065f46', // Teal text
                    borderRadius: '18px',
                    maxWidth: '85%', // Increased max width
                    fontSize: '14px', // Reduced font size
                    lineHeight: '1.4',
                    border: message.sender === 'ai' ? '1px solid #99f6e4' : 'none',
                    ...(message.sender === 'user'
                      ? { borderBottomRightRadius: '4px' }
                      : { borderBottomLeftRadius: '4px' })
                  }}
                >
                  {renderMessageWithLinks(message.text)}
                </div>
              </div>
            ))}
            {isTyping && (
              <div style={{ marginBottom: '12px', textAlign: 'left', display: 'flex', justifyContent: 'flex-start' }}>
                <div
                  style={{
                    display: 'inline-block',
                    padding: '10px 14px',
                    backgroundColor: '#e0e7ff',
                    borderRadius: '18px',
                    maxWidth: '85%',
                    fontSize: '14px',
                    border: '1px solid #99f6e4',
                    borderBottomLeftRadius: '4px'
                  }}
                >
                  <div style={{ display: 'flex', alignItems: 'center' }}>
                    <span style={{color: '#0d9488', fontWeight: '500'}}>Thinking</span>
                    <span style={{ marginLeft: '4px', color: '#0d9488' }}>.</span>
                    <span style={{ marginLeft: '4px', color: '#0d9488' }}>.</span>
                    <span style={{ marginLeft: '4px', color: '#0d9488' }}>.</span>
                  </div>
                </div>
              </div>
            )}
          </div>

          <form
            onSubmit={handleSendMessage}
            style={{
              padding: '12px',
              backgroundColor: 'white',
              borderTop: '1px solid #e5e7eb',
              display: 'flex'
            }}
          >
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask about Physical AI & Robotics..."
              style={{
                flex: 1,
                padding: '10px 16px', // Reduced padding
                border: '1px solid #e5e7eb',
                borderRadius: '20px',
                outline: 'none',
                fontSize: '14px', // Reduced font size
                transition: 'border-color 0.2s'
              }}
              onFocus={(e) => {
                e.target.style.borderColor = '#14b8a6';
              }}
              onBlur={(e) => {
                e.target.style.borderColor = '#e5e7eb';
              }}
            />
            <button
              type="submit"
              disabled={inputValue.trim() === ''}
              style={{
                marginLeft: '8px',
                backgroundColor: inputValue.trim() === '' ? '#d1d5db' : '#14b8a6',
                color: 'white',
                border: 'none',
                borderRadius: '50%',
                width: '40px',
                height: '40px',
                cursor: inputValue.trim() === '' ? 'not-allowed' : 'pointer',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                fontSize: '16px',
                transition: 'background-color 0.2s'
              }}
              onMouseOver={(e) => {
                if(inputValue.trim() !== '') {
                  e.target.style.backgroundColor = '#0d9488';
                }
              }}
              onMouseOut={(e) => {
                if(inputValue.trim() !== '') {
                  e.target.style.backgroundColor = '#14b8a6';
                }
              }}
            >
              ➤
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default SimpleAIAssistant;