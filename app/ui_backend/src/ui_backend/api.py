from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# TODO: make this modular
app = FastAPI()

# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5500",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class EchoIn(BaseModel):
    text: str

class EchoOut(BaseModel):
    echoed: str

@app.get("/api/test")
def get_test():
    return {"data": "test successful"}


