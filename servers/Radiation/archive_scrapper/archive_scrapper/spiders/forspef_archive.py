import scrapy


class ForspefArchiveScrapper(scrapy.Spider):

    name = "forspef"

    def start_requests(self):
        url = 'http://tromos.space.noa.gr/forspef/modules/'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):
        table = response.css("table.table-bordered")[0]
        datas = table.css("tr")
        date = response.css('div.text-center::text')
        date = date[1].extract()
        data = []
        for i in range(len(datas) - 1):
            data.append(datas[i + 1].css("td::text").extract())
        yield{
            "data": data, "date": date,
        }
