"""
FastAPI application entry point for the RAG Chatbot backend API.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import logging
from contextlib import asynccontextmanager
from .services.agent_service import AgentService
from ..config import Config

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan events for the FastAPI application.
    """
    logger.info("Starting up RAG Chatbot API...")
    # Initialize the AgentService once during startup
    try:
        config = Config()
        agent_service = AgentService(config)
        app.state.agent_service = agent_service
        logger.info("AgentService initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize AgentService: {str(e)}")
        raise
    yield
    # Shutdown events can be added here
    logger.info("Shutting down RAG Chatbot API...")

# Create FastAPI app instance with lifespan
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot with agent-driven response generation",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware to allow frontend to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify the exact frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import and include routers
from .routers.chat import router as chat_router
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])

@app.get("/")
def read_root():
    """Root endpoint for basic API information."""
    logger.info("Root endpoint accessed")
    return {"message": "RAG Chatbot API", "status": "running"}

@app.get("/health")
def health_check():
    """Health check endpoint."""
    logger.info("Health check endpoint accessed")
    return {"status": "healthy", "timestamp": __import__('datetime').datetime.now().isoformat()}

logger.info("FastAPI application initialized successfully with logging and lifespan events")