# example from: https://bommaritollc.com/2014/06/30/advanced-approximate-sentence-matching-python/

# Imports
import nltk.corpus
import nltk.tokenize
import nltk.stem.snowball
import string

# Get default English stopwords and extend with punctuation
stopwords = nltk.corpus.stopwords.words('english')
stopwords.extend(string.punctuation)
stopwords.append('')

tokenize = nltk.tokenize.word_tokenize

def jaccard_similarity(a, b, threshold=0.5):
    """Check if a and b are matches."""
    tokens_a = [token.lower().strip(string.punctuation) for token in tokenize(a) \
    if token.lower().strip(string.punctuation) not in stopwords]
    tokens_b = [token.lower().strip(string.punctuation) for token in tokenize(b) \
    if token.lower().strip(string.punctuation) not in stopwords]

    # Calculate Jaccard similarity
    ratio = len(set(tokens_a).intersection(tokens_b)) / float(len(set(tokens_a).union(tokens_b)))
    return ratio
