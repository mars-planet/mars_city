import scrapy


class CactusScrapper(scrapy.Spider):

    name = "cactus"

    def start_requests(self):
        url = 'http://www.sidc.oma.be/cactus/out/latestCMEs.html'
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
