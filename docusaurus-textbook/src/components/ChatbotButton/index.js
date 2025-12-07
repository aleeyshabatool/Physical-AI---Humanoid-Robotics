import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotButton.module.css';
import { 
  RiRobot2Line, 
  RiCloseLine, 
  RiSendPlaneLine, 
  RiUser3Line,
  RiSparkling2Line,
  RiChat3Line,
  RiArrowUpLine
} from 'react-icons/ri';

const ChatbotButton = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [message, setMessage] = useState('');
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Physical AI assistant. 🤖 Ready to help with robotics, ROS 2, digital twins, and AI integration.",
      isBot: true,
      timestamp: new Date(),
    }
  ]);
  const [isTyping, setIsTyping] = useState(false);
  
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 300);
    }
  }, [isOpen]);

  const handleSend = () => {
    if (!message.trim()) return;

    // Add user message
    const userMessage = {
      id: messages.length + 1,
      text: message,
      isBot: false,
      timestamp: new Date(),
    };
    setMessages([...messages, userMessage]);
    setMessage('');

    // Simulate typing
    setIsTyping(true);

    // Simulate bot response after delay
    setTimeout(() => {
      const responses = [
        "Great question! Let me check the textbook for the most accurate information about that topic.",
        "Based on the Physical AI textbook, I can explain that concept in detail. Would you like me to elaborate?",
        "That's covered in Chapter 3 about Digital Twins. The key points are simulation environments and real-time synchronization.",
        "For ROS 2 queries, check Chapter 2. It covers nodes, topics, and real-time communication in robotics systems.",
        "Hardware requirements are detailed in Chapter 6. You'll need RTX workstations and NVIDIA Jetson kits for AI processing.",
        "The VLA (Vision-Language-Action) system is explained in Chapter 5. It integrates Whisper, LLMs, and multimodal AI.",
        "I'm preparing a detailed response from the textbook resources. One moment please...",
        "That's an advanced topic! Let me fetch the relevant sections about AI-robot integration and neural networks."
      ];
      
      const botResponse = {
        id: messages.length + 2,
        text: responses[Math.floor(Math.random() * responses.length)],
        isBot: true,
        timestamp: new Date(),
      };
      
      setMessages(prev => [...prev, botResponse]);
      setIsTyping(false);
    }, 1500 + Math.random() * 1000);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const clearChat = () => {
    setMessages([
      {
        id: 1,
        text: "Chat cleared! How can I help you with Physical AI and robotics today?",
        isBot: true,
        timestamp: new Date(),
      }
    ]);
  };

  const quickQuestions = [
    "What is ROS 2?",
    "Explain digital twins",
    "Hardware requirements?",
    "AI in robotics?",
    "VLA systems?"
  ];

  return (
    <>
      {/* Floating Chat Button */}
      <button 
        className={`${styles.chatbotButton} ${isOpen ? styles.buttonActive : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? (
          <RiCloseLine className={styles.buttonIcon} />
        ) : (
          <>
            <RiRobot2Line className={styles.buttonIcon} />
            <span className={styles.notificationDot}></span>
          </>
        )}
      </button>

      {/* Chat Window */}
      <div className={`${styles.chatWindow} ${isOpen ? styles.windowOpen : ''}`}>
        {/* Header */}
        <div className={styles.chatHeader}>
          <div className={styles.headerLeft}>
            <RiRobot2Line className={styles.headerIcon} />
            <div className={styles.headerText}>
              <h3>Physical AI Assistant</h3>
              <span className={styles.status}>
                <RiSparkling2Line className={styles.statusIcon} />
                <span>Online</span>
              </span>
            </div>
          </div>
          <div className={styles.headerActions}>
            <button 
              className={styles.actionButton}
              onClick={clearChat}
              title="Clear chat"
            >
              Clear
            </button>
            <button 
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              <RiCloseLine />
            </button>
          </div>
        </div>
        
        {/* Chat Body */}
        <div className={styles.chatBody}>
          {/* Welcome Message */}
          <div className={styles.welcomeMessage}>
            <RiChat3Line className={styles.welcomeIcon} />
            <div className={styles.welcomeText}>
              <h4>Welcome to Physical AI Assistant</h4>
              <p>Ask me anything about robotics, AI, ROS 2, or the textbook content</p>
            </div>
          </div>

          {/* Quick Questions */}
          <div className={styles.quickQuestions}>
            <p className={styles.quickTitle}>Quick questions:</p>
            <div className={styles.quickButtons}>
              {quickQuestions.map((q, idx) => (
                <button
                  key={idx}
                  className={styles.quickButton}
                  onClick={() => setMessage(q)}
                >
                  {q}
                </button>
              ))}
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.map((msg) => (
              <div 
                key={msg.id} 
                className={`${styles.message} ${msg.isBot ? styles.botMessage : styles.userMessage}`}
              >
                <div className={styles.messageAvatar}>
                  {msg.isBot ? (
                    <RiRobot2Line className={styles.botAvatar} />
                  ) : (
                    <RiUser3Line className={styles.userAvatar} />
                  )}
                </div>
                <div className={styles.messageContent}>
                  <div className={styles.messageHeader}>
                    <span className={styles.sender}>
                      {msg.isBot ? 'AI Assistant' : 'You'}
                    </span>
                    <span className={styles.timestamp}>
                      {msg.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </span>
                  </div>
                  <div className={styles.messageText}>
                    {msg.text.split('\n').map((line, i) => (
                      <p key={i}>{line}</p>
                    ))}
                  </div>
                </div>
              </div>
            ))}
            
            {/* Typing Indicator */}
            {isTyping && (
              <div className={styles.typingIndicator}>
                <div className={styles.typingAvatar}>
                  <RiRobot2Line className={styles.botAvatar} />
                </div>
                <div className={styles.typingContent}>
                  <div className={styles.typingDots}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            
            <div ref={messagesEndRef} />
          </div>
        </div>

        {/* Chat Input */}
        <div className={styles.chatInput}>
          <div className={styles.inputWrapper}>
            <input
              ref={inputRef}
              type="text"
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about robotics, AI, ROS 2..."
              className={styles.messageInput}
              disabled={isTyping}
            />
            <button
              onClick={handleSend}
              className={styles.sendButton}
              disabled={!message.trim() || isTyping}
              aria-label="Send message"
            >
              <RiArrowUpLine className={styles.sendIcon} />
            </button>
          </div>
          <div className={styles.inputHint}>
            Press Enter to send • Shift+Enter for new line
          </div>
        </div>
      </div>
    </>
  );
};

export default ChatbotButton;