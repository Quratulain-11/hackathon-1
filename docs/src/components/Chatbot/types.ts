/**
 * Types for the Chatbot component
 */

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

export interface ChatRequest {
  query: string;
  top_k?: number;
  temperature?: number;
  max_tokens?: number;
}

export interface ChatResponse {
  answer: string;
  sources: any[]; // Array of DocumentSource objects from backend
  query: string;
  confidence: number;
  retrieval_time_ms: number;
  response_time_ms: number;
  timestamp: string;
  error?: string;
}

export interface BackendConfig {
  baseUrl: string;
}