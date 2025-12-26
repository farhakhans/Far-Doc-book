import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
import re
from bs4 import BeautifulSoup

# -------------------------------------
# CONFIG
# -------------------------------------
# Your Deployment Link:
SITEMAP_URL = "https://far-docusuraus.vercel.app/sitemap.xml"
COLLECTION_NAME = "humanoid_ai_book_two"

cohere_client = cohere.Client("1vdxQn9kxS0RplpHOh3qxptohf5X1kuuqJ8FcCNX")
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url="https://345bd403-87a1-4060-8fce-8acc703c142e.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.bKQ_w6lyFUnNMonSbwvC5Mdx8yCsejJt2wnlDnofxds"
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls[:10]:  # Print first 10 URLs
        print(" -", u)
    print(f"... and {len(urls)-10} more URLs" if len(urls) > 10 else "")

    return urls

# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    response = requests.get(url)
    html = response.text

    # For Docusaurus sites, content is typically within specific selectors
    soup = BeautifulSoup(html, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Try to find main content areas in Docusaurus
    content_selectors = [
        'main div[class*="docItem"]',  # Docusaurus doc item containers
        'article',  # Main article content
        'main',  # Main content area
        'div[class*="markdown"]',  # Markdown content containers
        'div[class*="container"]',  # Container elements
        'div[class*="content"]'  # Content containers
    ]

    text_content = ""
    for selector in content_selectors:
        elements = soup.select(selector)
        if elements:
            for element in elements:
                # Remove navigation and header elements that might be within content
                for unwanted in element(["nav", "header", "footer", "aside"]):
                    unwanted.decompose()
                text_content += element.get_text(separator=' ', strip=True) + "\n\n"
            break

    # If no specific content found, fall back to trafilatura
    if not text_content.strip():
        text_content = trafilatura.extract(html) or ""

    # Clean up the text
    text_content = re.sub(r'\n\s*\n', '\n\n', text_content)  # Remove excessive newlines
    text_content = re.sub(r'\s+', ' ', text_content)  # Normalize whitespace

    if not text_content.strip():
        print("[WARNING] No text extracted from:", url)

    return text_content

# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    chunks.append(text)
    return chunks

# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
        size=1024,        # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )

# -------------------------------------
# MAIN INGESTION PIPELINE - FOCUSED VERSION
# -------------------------------------
def ingest_book():
    # Focus on the most important documentation pages only
    important_urls = [
        "https://far-docusuraus.vercel.app/docs/intro",
        "https://far-docusuraus.vercel.app/docs/intro/overview",
        "https://far-docusuraus.vercel.app/docs/intro/learning-outcomes",
        "https://far-docusuraus.vercel.app/docs/modules/module-1-ros2/",
        "https://far-docusuraus.vercel.app/docs/modules/module-2-simulation/",
        "https://far-docusuraus.vercel.app/docs/modules/module-3-nvidia-isaac/",
        "https://far-docusuraus.vercel.app/docs/modules/module-4-vla/"
    ]

    print(f"Processing {len(important_urls)} important documentation pages...")

    create_collection()

    global_id = 1

    for i, url in enumerate(important_urls):
        print(f"\nProcessing ({i+1}/{len(important_urls)}): {url}")
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

    print("\nIngestion completed!")
    print("Total chunks stored:", global_id - 1)

if __name__ == "__main__":
    ingest_book()