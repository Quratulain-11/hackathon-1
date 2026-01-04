---
id: 3
title: fix-fastapi-request-parameter-bug
stage: red
date: 2025-12-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: agent-backend-api
branch: 3-agent-backend-api
user: user
command: /sp.implement
labels: ["bug-fix", "fastapi", "parameter", "request"]
links:
  spec: specs/3-agent-backend-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/rag_pipeline/api/routers/chat.py
 - deploy-chatbot/chatbot/src/rag_pipeline/api/routers/chat.py
tests:
 - null
---

## Prompt

```
You are debugging a FastAPI backend.

There is an error occurring when calling the chat endpoint:

Error:
'ChatRequest' object has no attribute 'app'

Context:
- This is a FastAPI application.
- ChatRequest is a Pydantic BaseModel used as the request body.
- The FastAPI app instance is stored in request.app.state.agent_service.
- The error happens when the chat endpoint tries to access request.app.

Task:
1. Locate the chat endpoint implementation (likely in src/rag_pipeline/api/routers/chat.py).
2. Identify where a parameter typed as ChatRequest is incorrectly used as if it were a FastAPI Request.
3. Fix the endpoint by:
   - Accepting a FastAPI Request object (from fastapi import Request)
   - Accepting ChatRequest as a separate parameter for the JSON body
4. Ensure request.app.state.agent_service is accessed only from the FastAPI Request object.
5. Ensure the user question is read from chat_request.question.
6. Do NOT rename the FastAPI app instance.
7. Return the corrected version of the chat endpoint code only.

Important:
- Do not introduce new architecture.
- Do not refactor unrelated files.
- Do not change business logic.
- Fix only the parameter handling bug.
```

## Response

Fixed the FastAPI parameter handling bug in the chat endpoint. The issue was that the function signature had `request: ChatRequest` but the code was trying to access `request.app.state.agent_service`, which only exists on the FastAPI Request object, not the Pydantic ChatRequest model.

The fix involved:
1. Adding `Request` import from fastapi
2. Changing the function signature from `chat_endpoint(request: ChatRequest)` to `chat_endpoint(request: Request, chat_request: ChatRequest)`
3. Updating all query parameter references to use `chat_request` instead of `request` (e.g., `chat_request.query`, `chat_request.top_k`, etc.)
4. Keeping the access to `request.app.state.agent_service` using the FastAPI Request object

Both the main backend and deploy-chatbot versions were updated to fix the same issue.

## Outcome

- ✅ Impact: Fixed the 'ChatRequest' object has no attribute 'app' error
- 🧪 Tests: null
- 📁 Files: backend/src/rag_pipeline/api/routers/chat.py, deploy-chatbot/chatbot/src/rag_pipeline/api/routers/chat.py
- 🔁 Next prompts: /sp.tasks, /sp.test
- 🧠 Reflection: Parameter typing bug resolved by properly separating FastAPI Request from Pydantic model

## Evaluation notes (flywheel)

- Failure modes observed: FastAPI parameter naming conflict
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null