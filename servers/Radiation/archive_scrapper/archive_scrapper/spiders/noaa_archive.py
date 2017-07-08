import scrapy
from scrapy.http import Request


class FtpSpider(scrapy.Spider):
    name = "NOAA"
    #allowed_domains = ["ftp.swpc.noaa.gov/pub/warehouse/"]
    handle_httpstatus_list = [404]
    count = 2007
    string = 'ftp://ftp.swpc.noaa.gov/pub/warehouse/'

    def start_requests(self):
    	
    	urls = ['ftp://ftp.swpc.noaa.gov/pub/warehouse/2007/2007_DPD.txt']
    	for url in urls:
        	yield Request(url,meta={'ftp_user': 'anonymous', 'ftp_password': ''})
    def parse(self, response):
        datas = response.css("p::text").extract()
        data = []
        for i in datas:
            i = i.split('\n')
            d = i
            data.append(d)
        data = data[0][11:]
        for i,v in enumerate(data):
        	data[i] = data[i].split('  ')
        	#for idx,val in enumerate(data[i]):
        		#if data[i][idx] == '':
        		#	del(data[i][idx])
        for idx,val in enumerate(data):
        	data[idx] = ' '.join(data[idx]).split()
        print data
        if self.count < 2017:
        	self.count = self.count + 1
        	url = self.string + str(self.count) + '/' + str(self.count) + '_DPD.txt'
        	yield scrapy.Request(url, callback = self.parse,meta={'ftp_user': 'anonymous', 'ftp_password': ''})
