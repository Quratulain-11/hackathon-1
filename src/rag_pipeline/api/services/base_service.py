"""
Base service for the RAG Chatbot API services.
"""
import logging
from abc import ABC, abstractmethod
from typing import Any, Optional
from ...config import Config
from ...storage.qdrant_client import QdrantClient
from ...embeddings.cohere_client import CohereClient


class BaseService(ABC):
    """
    Base service class that provides common functionality for all API services.
    """

    def __init__(self, config: Config):
        self.config = config
        self.qdrant_client = QdrantClient(config)
        self.cohere_client = CohereClient(config)
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def execute(self, *args, **kwargs) -> Any:
        """
        Execute the service operation.
        """
        pass


class ServiceError(Exception):
    """
    Base exception for service errors.
    """
    def __init__(self, message: str, error_code: str = "SERVICE_ERROR", details: Optional[dict] = None):
        self.message = message
        self.error_code = error_code
        self.details = details or {}
        super().__init__(self.message)