import scrapy

class CactusArchiveScrapper(scrapy.Spider):

    name = "cactus"
    count = 0

    def start_requests(self):
        url = 'https://secchi.nrl.navy.mil/cactus/'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):
        table = response.css("pre::text")
        data = []
        for i in range(len(table) - 2):
            datas = table[i + 2].extract()
            data.append(datas.split('|')[1:9])
            if 'Flow' in datas:
                break
        yield{
            'data': data,
        }
        linkobj = response.css('li')[2:24]
        links = linkobj.css("a::attr(href)").extract()
        if self.count<len(links):
        	self.count = self.count + 1
        	yield response.follow(links[self.count], self.parse)

