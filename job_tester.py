#! /usr/bin/python3

import random
import requests
import sys
from typing import List
from urllib.error import HTTPError


WAYPOINTS: List[int] = [0, 1, 2, 3, 4]
SMARTCITY_JOB_ENDPOINT: str = 'http://smartcity.ddns.net:8081/job/execute/{}/{}/{}'

def usage():
	print('./job_tester [NUM_JOBS]')


def main(args: List[str]):
	if len(args) < 2:
		usage()
		return -1

	num_jobs: int = int(args[1])

	for i in range(num_jobs):
		start: int = random.choice(WAYPOINTS)
		end: int = random.choice(WAYPOINTS)

		request_endpoint: str = SMARTCITY_JOB_ENDPOINT.format(start, end, i)

		try:
			response: str = requests.post(request_endpoint).text
			print(response)
		except HTTPError as he:
			print('Failed to make request: HTTP {}: {}'.format(he.code, he.reason))

if __name__ == '__main__':
	main(sys.argv)