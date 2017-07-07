import scrapy


class PrediccsArchiveScrapper(scrapy.Spider):

    name = "prediccs"

    def start_requests(self):
        url = 'http://prediccs.sr.unh.edu/data/goesPlots/archive/'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):
        datas = response.css("p::text").extract_first()
        datas = datas.split("\n")[12:]
        data = []
        for i in datas:
            i = i.split('\t')
            d = i[:6]
            d.append(i[12])
            data.append(d)
        yield{
            'data': data,
        }
