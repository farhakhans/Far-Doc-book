# Chatbot Server Setup Instructions

## Starting the Backend Server

To get your RAG chatbot working, you need to start the backend server:

1. Open a new terminal/command prompt
2. Navigate to your project directory: `cd C:\Users\Hp\Desktop\Far-Docusuraus`
3. Run the server with: `node server.js`
4. The server should start on port 3001

## Alternative Server Start Commands

If you have npm scripts configured, you might also be able to use:
- `npm start`
- `npm run server`
- `npx nodemon server.js` (for auto-restarting on changes)

## Checking if Server is Running

- The server should output "Server running on port 3001"
- You can verify by visiting: http://localhost:3001/api/health in your browser
- You should see a JSON response with status information

## Troubleshooting

If you encounter issues:

1. **Port already in use**: Make sure no other process is using port 3001
2. **Python dependencies**: Ensure you have Python installed and the required packages for RAG-DOCS
3. **File permissions**: Make sure you have read/write access to the project directory
4. **Missing dependencies**: Run `npm install` to ensure all Node.js dependencies are installed

## Required Dependencies

Make sure you have:
- Node.js (v14 or higher)
- Python (with required packages for RAG functionality)
- All npm packages installed via `npm install`

## Development Notes

The chatbot will now show a visual indicator when the server is offline, and will prevent sending messages when the server is unavailable. Once you start the server, the chatbot should work properly with your RAG system.