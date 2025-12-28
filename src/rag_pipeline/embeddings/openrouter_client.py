import openai
from typing import List, Optional
import logging
import time
import random
from ..config import Config


logger = logging.getLogger(__name__)


class OpenRouterClient:
    """
    Wrapper around OpenRouter client to handle chat completions.
    """
    def __init__(self, config: Config):
        self.config = config
        # Initialize OpenRouter client with the OpenAI library using OpenRouter base URL
        self.client = openai.OpenAI(
            api_key=config.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

    def generate_response(self, prompt: str, max_tokens: int = 500, temperature: float = 0.7, max_retries: int = 3) -> Optional[str]:
        """
        Generate a response using OpenRouter with retry logic and timeout handling.

        Args:
            prompt: The input prompt to generate response for
            max_tokens: Maximum tokens for the response
            temperature: Response randomness (0.0 to 1.0)
            max_retries: Maximum number of retry attempts

        Returns:
            Generated response text, or None if failed
        """
        for attempt in range(max_retries):
            try:
                start_time = time.time()

                # Generate response using OpenRouter
                response = self.client.chat.completions.create(
                    model="xiaomi/mimo-v2-flash:free",  # Using the provided free model
                    messages=[
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=max_tokens,
                    temperature=temperature,
                )

                elapsed_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                response_text = response.choices[0].message.content

                logger.info(f"Generated response successfully in {elapsed_time:.2f}ms")
                return response_text

            except openai.APIError as e:
                elapsed_time = (time.time() - start_time) * 1000 if 'start_time' in locals() else 0
                logger.error(f"OpenRouter API error (attempt {attempt + 1}, {elapsed_time:.2f}ms): {str(e)}")

                # Check if it's a rate limit error
                if "rate limit" in str(e).lower() or "too many requests" in str(e).lower() or "429" in str(e).lower():
                    # Exponential backoff with jitter
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Rate limited. Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue
                elif "connection" in str(e).lower() or "timeout" in str(e).lower():
                    # Connection or timeout error - wait and retry
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Connection error. Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue
                else:
                    # Other OpenRouter-specific error, don't retry
                    logger.error(f"Non-retryable OpenRouter error: {str(e)}")
                    break
            except Exception as e:
                elapsed_time = (time.time() - start_time) * 1000 if 'start_time' in locals() else 0
                logger.error(f"Unexpected error generating response (attempt {attempt + 1}, {elapsed_time:.2f}ms): {str(e)}")

                if attempt == max_retries - 1:  # Last attempt
                    logger.error("Max retries reached, giving up on response generation")
                    break
                else:
                    # Exponential backoff with jitter for general errors
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue

        logger.error("Failed to generate response after all retry attempts")
        return None