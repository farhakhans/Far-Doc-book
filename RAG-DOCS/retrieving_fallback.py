import json
import sys

def format_response(query, results):
    """Format the response in a structured way with short, point-to-point answers"""
    if results:
        # Create a focused response with short, direct answers
        query_lower = query.lower()

        # Split query into keywords, removing common words for better matching
        query_keywords = [word for word in query_lower.split() if word not in ['what', 'is', 'the', 'a', 'an', 'and', 'or', 'of', 'in', 'on', 'at', 'to', 'for', 'with', 'by']]

        # Only add query keywords that are meaningful
        query_keywords = [kw for kw in query_keywords if len(kw) > 2]  # Only keywords longer than 2 chars

        # Extract the most relevant sentences that match the query
        relevant_sentences = []
        for result in results:
            sentences = result.split('. ')
            for sentence in sentences:
                # Check if sentence contains any query keywords
                if any(kw in sentence.lower() for kw in query_keywords) and len(sentence.strip()) > 10:
                    relevant_sentences.append(sentence.strip())
                    if len(relevant_sentences) >= 2:  # Limit to 2 sentences for brevity
                        break
            if len(relevant_sentences) >= 2:  # Stop after finding 2 relevant sentences
                break

        if relevant_sentences:
            # Create short, point-to-point answers
            response_text = "Answer: " + "; ".join(relevant_sentences[:2]) + "."
        else:
            # If no keyword matches found, return the first few sentences of the first result
            first_few_sentences = results[0].split('. ')[:2]
            response_text = "Answer: " + "; ".join([s.strip() for s in first_few_sentences if s.strip()]) + "."

        # Create sources
        sources = []
        for i, result in enumerate(results[:2]):  # Use top 2 results as sources
            sources.append({"title": f"Source {i+1}", "url": f"/docs/intro#{i+1}"})

    else:
        response_text = "No relevant information found in the robotics book for your query."
        sources = []

    return {
        "query": query,
        "response": response_text,
        "sources": sources
    }

# Fallback responses for when external APIs are not available
FALLBACK_RESPONSES = {
    "robotics": "Answer: Robotics is an interdisciplinary field involving engineering and science that focuses on design, construction, operation, and application of robots.",
    "ai": "Answer: Artificial Intelligence (AI) refers to the simulation of human intelligence in machines programmed to think and learn like humans.",
    "machine learning": "Answer: Machine learning is a subset of AI that enables computers to learn and improve from experience without being explicitly programmed.",
    "automation": "Answer: Automation refers to technology that uses programmed devices to execute specific functions with minimal human intervention.",
    "control systems": "Answer: Control systems manage, command, direct, or regulate the behavior of other devices or systems.",
    "sensors": "Answer: Sensors are devices that detect and respond to physical inputs from the environment, converting them into signals for electronic systems.",
    "actuators": "Answer: Actuators are components that move and control a mechanism or system by converting energy into motion.",
    "kinematics": "Answer: Kinematics is the branch of mechanics that describes motion without considering its causes.",
    "dynamics": "Answer: Dynamics is the branch of mechanics concerned with the motion of objects and the forces that cause this motion.",
    "path planning": "Answer: Path planning is the computational problem of finding a feasible path from a start to a goal position.",
    "computer vision": "Answer: Computer vision is a field of artificial intelligence that trains computers to interpret and understand the visual world.",
    "robot navigation": "Answer: Robot navigation is the process by which a robot determines how to move from one location to another.",
}

def get_fallback_response(query):
    """Provide a fallback response when external APIs are unavailable"""
    query_lower = query.lower()

    # Check if any key terms from fallback responses are in the query
    for term, response in FALLBACK_RESPONSES.items():
        if term in query_lower:
            return {
                "query": query,
                "response": response,
                "sources": [{"title": "System Fallback", "url": "/docs/intro"}]
            }

    # If no specific term matches, provide a general response
    return {
        "query": query,
        "response": "The system is currently experiencing connectivity issues. Please try again later.",
        "sources": [{"title": "System Fallback", "url": "/docs/intro"}]
    }

# When called as a script with command line arguments
if __name__ == "__main__":
    if len(sys.argv) > 1:
        query = sys.argv[1]
        try:
            # For this fallback script, we'll just return a predefined response
            # In a real implementation, you would have local data to search through
            response = get_fallback_response(query)
            print(json.dumps(response))
        except Exception as e:
            error_response = {
                "query": query,
                "response": f"Error processing query: {str(e)}",
                "sources": []
            }
            print(json.dumps(error_response))
    else:
        # For testing purposes
        response = get_fallback_response("What is robotics?")
        print(json.dumps(response))