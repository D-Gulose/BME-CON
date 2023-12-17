#!/usr/bin/env python3

import subprocess, os, atexit, requests, traceback, socket, http.client, contextlib, functools, time, stat
from sys import argv, exit, stdout as sys_stdout
from time import sleep
from signal import SIGINT
from hashlib import sha512
from glob import glob

print('Waiting for g++ build (timeout 30s)...', end='')
try:
    gcc = subprocess.run(
        ['g++','-std=c++17','-g','-Wall','-Werror'] + glob('*.cpp') + ['-o','server'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        timeout=30
    )
    if gcc.returncode == 0:
        print('OK.')
    else:
        print('failed.\n')
        print(gcc.stdout)
        print('\n== ABORT: gcc exited with nonzero code (%d) ==' % gcc.returncode)
        exit(1)
except subprocess.TimeoutExpired as e:
    print('timed out.\n')
    print(e.stdout)
    print('\n== ABORT: Timeout (30s) hit while waiting for g++ build ==')
    exit(1)
    

proc = subprocess.Popen(
    ['valgrind','--leak-check=full','--show-leak-kinds=all','--show-error-list=yes','./server'],
    stdout = subprocess.PIPE, stderr = subprocess.PIPE, text=True,
)
atexit.register(lambda p: p.kill(), proc)

print('Waiting for server to come up (5s)...')
sleep(5)
print('Checking if server survived startup... ', end='')
if proc.poll() is not None:
    print('it terminated.\n')
    
    try:
        (stdout, stderr) = proc.communicate(timeout=5)
        print(stdout)
        print('')
        print(stderr)
        print('\n== ABORT: Server process did not survive startup (exit code %d) ==' % proc.returncode)
    except subprocess.TimeoutExpired:
        print(e.stdout)
        print('')
        print(e.stderr)
        print('\n== ABORT: Server process did not survive startup ==')
    
    exit(1)

print('OK.')

print('Now starting test run.', end='\n\n')

# TEST FRAMEWORK STARTS HERE 

class _TestFailedException(Exception):
    def __init__(self, text):
        self.text = text
    def __str__(self):
        return self.text

class _TestSkippedException(Exception):
    pass
    
def _statusstr(code, serverstr=None):
    try:
        ourstr = http.client.responses[code]
    except KeyError:
        ourstr = 'Unknown?'
    if serverstr is not None and ourstr != serverstr:
        return '%s, server says "%s"' % (ourstr, serverstr)
    else:
        return ourstr
    
_Tests = []
_CurrentSection = None

def SECTION_START(name):
    global _CurrentSection
    _CurrentSection = []
    _Tests.append((name, _CurrentSection))

def Test(fn):
    _CurrentSection.append(fn)

def Fail(why):
    raise _TestFailedException(why)

def AssertEquals(desc, has, expect):
    if has != expect:
        raise _TestFailedException('Assertion of "%s" failed.\nObserved: %s\nExpected: %s' % (desc, repr(has), repr(expect)))

def AssertStatus(req, target):
    if req.status_code != target:
        raise _TestFailedException('Expected HTTP status %d (%s), but got %d (%s)' %
                                    (target, _statusstr(target), req.status_code, _statusstr(req.status_code, req.reason)))

def AssertStatusClientError(req):
    if req.status_code not in range(400,500):
        Fail('Expected 4xx HTTP status (client error), got %d (%s)' % (req.status_code, _statusstr(req.status_code, req.reason)))

def AssertHeader(req, key, value):
    try:
        present = req.headers[key]
        if present != str(value):
            raise _TestFailedException('HTTP header field "%s": expected "%s", got "%s".' % (key, value, present))
    except KeyError:
        raise _TestFailedException('Expected HTTP "%s" header (with value "%s"). Header is not present.' % (key, value))

def GetHeaderAssertExists(req, key):
    try:
        return req.headers[key]
    except KeyError:
        raise _TestFailedException('Expected HTTP "%s" header to exist. Header does not exist.' % key)

def Skip():
    raise _TestSkippedException

def SkipIf(c):
    if c:
        Skip()

# END OF TEST FRAMEWORK, ACTUAL TESTS BELOW

SECTION_START('Assignment sheet')

hasRangeSupport = False
@Test
def BasicHTTPRequest():
    with requests.get('http://127.0.0.1:8000/lorem.txt') as resp:
        AssertStatus(resp, 200)
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-type', 'text/plain')
        AssertHeader(resp, 'content-length', 17)
        AssertEquals('body', resp.content, b'lorem ipsum dolor')
        
        if 'accept-ranges' in resp.headers:
            global hasRangeSupport
            hasRangeSupport = True

@Test
def BasicErrorResponse():
    with requests.get('http://127.0.0.1:8000/iorem.txt') as resp:
        AssertStatus(resp, 404)

@Test
def RequestSubfolderIndexDocument():
    with requests.get('http://127.0.0.1:8000/foo/') as resp:
        AssertStatus(resp, 200)
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-length', 50)
        AssertHeader(resp, 'content-type', 'text/html')
        AssertEquals('body', resp.content, b'<html><head><title>Foo index</title></head></html>')

@Test
def FullySatisfiableRangeRequest():
    SkipIf(not hasRangeSupport)
    with requests.get('http://127.0.0.1:8000/lorem.txt', headers={ 'range': 'bytes=4-9' }) as resp:
        AssertStatus(resp, 206)
        AssertHeader(resp, 'accept-ranges', 'bytes')
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-length', 6)
        AssertHeader(resp, 'content-range', 'bytes 4-9/17')
        AssertEquals('body', resp.content, b'm ipsu')

@Test
def PartiallySatisfiableRangeRequest():
    SkipIf(not hasRangeSupport)
    with requests.get('http://127.0.0.1:8000/lorem.txt', headers={ 'range': 'bytes=10-19' }) as resp:
        AssertStatus(resp, 206)
        AssertHeader(resp, 'accept-ranges', 'bytes')
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-length', 7)
        AssertHeader(resp, 'content-range', 'bytes 10-16/17')
        AssertEquals('body', resp.content, b'm dolor')

@Test
def OutOfBoundsRangeRequest():
    SkipIf(not hasRangeSupport)
    with requests.get('http://127.0.0.1:8000/lorem.txt', headers={ 'range': 'bytes=20-29' }) as resp:
        AssertStatus(resp, 416)
        AssertHeader(resp, 'content-range', 'bytes */17')

@Test
def SuffixRangeRequest1():
    SkipIf(not hasRangeSupport)
    with requests.get('http://127.0.0.1:8000/lorem.txt', headers={ 'range': 'bytes=8-' }) as resp:
        AssertStatus(resp, 206)
        AssertHeader(resp, 'accept-ranges', 'bytes')
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-length', 9)
        AssertHeader(resp, 'content-range', 'bytes 8-16/17')
        AssertEquals('body', resp.content, b'sum dolor')

@Test
def SuffixRangeRequest1():
    SkipIf(not hasRangeSupport)
    with requests.get('http://127.0.0.1:8000/lorem.txt', headers={ 'range': 'bytes=-8' }) as resp:
        AssertStatus(resp, 206)
        AssertHeader(resp, 'accept-ranges', 'bytes')
        AssertHeader(resp, 'connection', 'close')
        AssertHeader(resp, 'content-length', 8)
        AssertHeader(resp, 'content-range', 'bytes 9-16/17')
        AssertEquals('body', resp.content, b'um dolor')


SECTION_START('Your own tests  ')

# END OF TEST DEFINITIONS

_TestResults = []

for (sectionName, testData) in _Tests:
    print('== %s ==' % sectionName.strip())
    numStr = str(len(testData))
    fmtStr = ('[%%%dd/%s] "%%s"... ' % (len(numStr), numStr))
    
    nPass = 0
    nFail = 0
    nSkip = 0
    
    if not testData:
        print('(no tests in this section)')

    for (idx, testFn) in enumerate(testData, 1):
        testName = testFn.__name__
        print(fmtStr % (idx, testName), end='')
        sys_stdout.flush()
        try:
            testFn()
            print('OK.')
            nPass += 1
        except _TestFailedException as e:
            print('\x1b[2K\r\n(!!) ', end='')
            print(fmtStr % (idx, testName), end='FAIL.\n')
            print(str(e), end='\n\n')
            nFail += 1
        except _TestSkippedException:
            print('SKIP.')
            nSkip += 1
        except Exception:
            print('\x1b[2K\r\n(!!) ', end='')
            print(fmtStr % (idx, testName), end='FAIL.\n')
            traceback.print_exc()
            print('')
            nFail += 1
    
    _TestResults.append((sectionName, (nPass, nFail, nSkip)))
    print('')

print('Done running tests, terminating server... ', end='')
proc.send_signal(SIGINT)
try:
    (stdout, stderr) = proc.communicate(timeout=30)
    print('OK.\n')
    print('Valgrind output below:')
    print(stderr)
    print('\n== Test summary: ==')
    for (category, (nPass, nFail, nSkip)) in _TestResults:
        print('%s: %2d passed, %2d failed, %2d skipped' % (category, nPass, nFail, nSkip))
except subprocess.TimeoutExpired:
    print('timed out?\n')
    print(e.stdout)
    print('')
    print(e.stderr)
    print('\n== ABORT: Server process failed to terminate on SIGINT ==')
    exit(1)
