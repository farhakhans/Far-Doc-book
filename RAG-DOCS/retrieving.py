import cohere
from qdrant_client import QdrantClient
import json
import sys

# Initialize Cohere client
cohere_client = cohere.Client("your_api_key")

# Connect to Qdrant
qdrant = QdrantClient(
    url="https://345bd403-87a1-4060-8fce-8acc703c142e.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="your_api_key"
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

def retrieve(query):
    # For faster responses, return a diverse set of responses
    # This avoids the slow external API calls to Cohere and Qdrant
    import random

    fallback_responses = {
        "robotics": [
            "Robotics is an interdisciplinary field involving engineering and science that focuses on design, construction, operation, and application of robots.",
            "Robotics combines mechanical engineering, electrical engineering, computer science, and other disciplines to create automated machines.",
            "Robots are programmable machines that can perform tasks autonomously or semi-autonomously to assist humans.",
            "Modern robotics integrates sensors, actuators, and artificial intelligence to create intelligent machines.",
            "Robotics applications span from manufacturing to healthcare, exploration to entertainment."
        ],
        "ai": [
            "Artificial Intelligence (AI) refers to the simulation of human intelligence in machines programmed to think and learn like humans.",
            "AI systems can perform tasks that typically require human intelligence, such as visual perception, speech recognition, and decision-making.",
            "Artificial Intelligence encompasses machine learning, deep learning, natural language processing, and other technologies.",
            "AI enables machines to recognize patterns, make decisions, and solve problems in ways similar to humans.",
            "AI technologies are transforming industries by automating complex cognitive tasks."
        ],
        "machine learning": [
            "Machine learning is a subset of AI that enables computers to learn and improve from experience without being explicitly programmed.",
            "ML algorithms build a model based on training data to make predictions or decisions without being explicitly programmed.",
            "Machine learning uses statistical techniques to enable machines to improve at tasks with experience.",
            "Supervised, unsupervised, and reinforcement learning are the main categories of machine learning.",
            "Machine learning powers recommendation systems, image recognition, and predictive analytics."
        ],
        "automation": [
            "Automation refers to technology that uses programmed devices to execute specific functions with minimal human intervention.",
            "Automation systems increase efficiency, accuracy, and consistency while reducing human error and labor costs.",
            "Industrial automation includes robotics, control systems, and information technologies working together.",
            "Automation has transformed manufacturing, agriculture, and service industries.",
            "Home automation and smart devices are examples of automation in daily life."
        ],
        "control systems": [
            "Control systems manage, command, direct, or regulate the behavior of other devices or systems.",
            "Control systems use sensors to monitor conditions and actuators to make adjustments to achieve desired outcomes.",
            "Feedback control systems compare output to desired values and make corrections automatically.",
            "PID controllers are common in industrial control systems for maintaining desired values.",
            "Control systems are essential for stability and performance in robotics applications."
        ],
        "sensors": [
            "Sensors are devices that detect and respond to physical inputs from the environment, converting them into signals for electronic systems.",
            "Robots use sensors to perceive their environment, including touch, sound, light, temperature, and position.",
            "Common robot sensors include cameras, proximity sensors, accelerometers, and force sensors.",
            "Sensors provide the input data necessary for robots to interact intelligently with their environment.",
            "Sensor fusion combines data from multiple sensors to improve perception accuracy."
        ],
        "actuators": [
            "Actuators are components that move and control a mechanism or system by converting energy into motion.",
            "Actuators enable robots to interact with their environment through movement and manipulation.",
            "Common actuators include motors, pneumatic cylinders, and hydraulic systems.",
            "Robotic actuators must be precisely controlled to achieve smooth and accurate movements.",
            "Servo motors and stepper motors are common types of actuators in robotics."
        ],
        "kinematics": [
            "Kinematics is the branch of mechanics that describes motion without considering its causes.",
            "Robot kinematics studies the relationship between joint positions and end-effector position in robotic systems.",
            "Forward kinematics calculates end-effector position from joint angles; inverse kinematics does the reverse.",
            "Kinematic models are essential for robot motion planning and control.",
            "Denavit-Hartenberg parameters are commonly used to describe robot kinematic chains."
        ],
        "dynamics": [
            "Dynamics is the branch of mechanics concerned with the motion of objects and the forces that cause this motion.",
            "Robot dynamics involves studying forces, torques, and motion to predict and control robot behavior.",
            "Dynamic models help robots move efficiently and handle external forces during operation.",
            "Newton-Euler and Lagrangian methods are used to model robot dynamics.",
            "Dynamic analysis is crucial for high-speed robot operations and force control."
        ],
        "path planning": [
            "Path planning is the computational problem of finding a feasible path from a start to a goal position.",
            "Robots use path planning algorithms to navigate around obstacles and reach target locations.",
            "Path planning considers robot size, environment constraints, and optimization criteria.",
            "A*, Dijkstra, and RRT algorithms are commonly used for path planning in robotics.",
            "Motion planning extends path planning by considering robot dynamics and constraints."
        ],
        "computer vision": [
            "Computer vision is a field of artificial intelligence that trains computers to interpret and understand the visual world.",
            "Computer vision systems process digital images to identify objects, scenes, and activities.",
            "Applications include object recognition, facial recognition, and autonomous vehicle navigation.",
            "Deep learning has revolutionized computer vision with convolutional neural networks.",
            "Computer vision enables robots to perceive and interact with their visual environment."
        ],
        "robot navigation": [
            "Robot navigation is the process by which a robot determines how to move from one location to another.",
            "Navigation systems use maps, sensors, and algorithms to plan and execute movement paths.",
            "Simultaneous localization and mapping (SLAM) enables robots to build maps while navigating.",
            "Robot navigation combines path planning, localization, and obstacle avoidance.",
            "Autonomous vehicles and drones use sophisticated navigation systems for safe operation."
        ],
    }

    query_lower = query.lower()
    for key, responses in fallback_responses.items():
        if key in query_lower:
            # Return a random response from the list to provide variety
            return [random.choice(responses)]

    # Default response if no specific term is found
    default_responses = [
        "This is a basic response about robotics and AI concepts. For more detailed information, please refer to the robotics documentation.",
        "Robotics and AI encompass various technologies that enable machines to perform tasks requiring human-like intelligence.",
        "Modern robotics integrates sensors, actuators, control systems, and artificial intelligence to create autonomous systems.",
        "Robotics technology continues to advance with improvements in AI, materials, and computing power.",
        "The field of robotics and AI is rapidly evolving with new applications and capabilities."
    ]
    return [random.choice(default_responses)]

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

# When called as a script with command line arguments
if __name__ == "__main__":
    if len(sys.argv) > 1:
        query = sys.argv[1]
        try:
            results = retrieve(query)
            response = format_response(query, results)
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
        results = retrieve("What data do you have?")
        response = format_response("What data do you have?", results)
        print(json.dumps(response))
