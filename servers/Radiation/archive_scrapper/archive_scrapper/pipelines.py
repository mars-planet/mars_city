import pymongo


class ScrapperPipeline(object):
    def process_item(self, item, spider):
        return item


class MongoPipeline(object):
    def __init__(self, mongo_uri, mongo_db):
        self.mongo_uri = mongo_uri
        self.mongo_db = mongo_db

    @classmethod
    def from_crawler(cls, scrapper):
        return cls(
            mongo_uri=scrapper.settings.get('MONGO_URI'),
            mongo_db=scrapper.settings.get('MONGO_DATABASE')
        )

    def open_spider(self, spider):
        self.client = pymongo.MongoClient(self.mongo_uri)
        self.db = self.client[self.mongo_db]

    def close_spider(self, spider):
        self.client.close()

    def process_item(self, item, spider):

        if spider.name == "cactus":
            self.db['cactus'].insert(dict(item))
            return item

        if spider.name == "prediccs":
            if self.db['prediccs'].count() == 0:
                self.db['prediccs'].insert(dict(item))
                return item
            for i in self.db['prediccs'].find().sort('_id', -1).limit(1):
                last_entry = i
            if last_entry['data'][-1][0] != item['data'][-1][0]:
                self.db['prediccs'].insert(dict(item))
            return item

        if spider.name == "forspef":

            if self.db['forspef'].count() == 0:
                self.db['forspef'].insert(dict(item))
                return item
            for i in self.db['forspef'].find().sort('_id', -1).limit(1):
                last_entry = i
            if last_entry['date'] != item['date']:
                self.db['forspef'].insert(dict(item))
            return item
